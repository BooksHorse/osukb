#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embedded_hal::timer::CountDown;
mod touchio;
use crate::touchio::Touchio;

use rp2040_hal::prelude;

#[rtic::app(
    device = rp2040_hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {

    use fugit::ExtU32;
    use rp2040_hal::gpio::bank0::Gpio25;
    use rp2040_hal::gpio::{FunctionNull, FunctionSio, PullDown, SioOutput};
    use usbd_human_interface_device::descriptor::InterfaceProtocol;
    use usbd_human_interface_device::device::keyboard::{
        BootKeyboardConfig, BOOT_KEYBOARD_REPORT_DESCRIPTOR,
    };
    use usbd_human_interface_device::interface::{InterfaceBuilder, ManagedIdleInterfaceConfig};
    use usbd_human_interface_device::page::Keyboard;
    use usbd_human_interface_device::prelude::*;

    use core::borrow::BorrowMut;
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use panic_probe as _;

    use rtt_target::{debug_rprintln, debug_rtt_init_print};
    // use rp2040_hal as halpi;
    use usb_device::{class_prelude::*, prelude::*};
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    use crate::touchio::Touchio;
    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_pin: rp2040_hal::gpio::Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
    }

    #[init(local=[
        // Task local initialized resources are static
        // Here we use MaybeUninit to allow for initialization in init()
        // This enables its usage in driver initialization
        // i2c_ctx: core::mem::MaybeUninit<I2CBus>  = core::mem::MaybeUninit::uninit()

    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        debug_rtt_init_print!();
        let mut pac = rp2040_hal::pac::Peripherals::take().unwrap();
        let core = rp2040_hal::pac::CorePeripherals::take().unwrap();

        let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

        let clocks = rp2040_hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        rtic_monotonics::rp2040::Timer::start(pac.TIMER, &pac.RESETS, rp2040_timer_token); // default rp2040 clock-rate is 125MHz

        let mut sio = rp2040_hal::Sio::new(pac.SIO);
        let pins = rp2040_hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));

        let mut keyboard = UsbHidClassBuilder::new()
            .add_device(BootKeyboardConfig::new(ManagedIdleInterfaceConfig::new(
                InterfaceBuilder::new(BOOT_KEYBOARD_REPORT_DESCRIPTOR)
                    .unwrap()
                    .description("bocchi")
                    .boot_device(InterfaceProtocol::Keyboard)
                    .idle_default(10.millis())
                    .unwrap()
                    .in_endpoint(1.millis())
                    .unwrap()
                    .build(),
            )))
            .build(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
            .manufacturer("BooksHorse")
            .product("Bocchi The Board!")
            .serial_number("BOCCHI")
            .build();

        // Set up the USB Communications Class Device driver
        //let mut serial = SerialPort::new(bus_ref);

        // Create a USB device with a fake VID and PID
        // let mut usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        //     .manufacturer("Fake company")
        //     .product("Serial Port")
        //     .serial_number("TEST")
        //     .device_class(2) // from: https://www.usb.org/defined-class-codes
        //     .build();

        // unsafe {
        //     // Enable the USB interrupt
        //     //core.NVIC.set_priority(interrupt::USBCTRL_IRQ, 1);
        //     pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        // };

        let mut led_pin = pins.gpio25.into_push_pull_output();
        let mut _button_pin = pins.gpio24.into_pull_up_input();
        let mut binding_a = pins.gpio11;
        let mut binding_b = pins.gpio6;
        let mut a = Touchio::new(binding_a.into());
        let mut b: Touchio<rp2040_hal::gpio::bank0::Gpio11, FunctionNull, PullDown> =
            Touchio::new(binding_b);

        (
            Shared {},
            Local {
                led_pin,
                // i2c: i2c_tmp,
            },
        )
    }

    #[task(local = [led_pin])]
    async fn keyboard_task(ctx: keyboard_task::Context) {
        loop {
            let led_pin = ctx.local.led_pin;
            // if tick_osuclick.wait().is_ok() {
            //  critical_section::with(|_| {
            // push_kb_movement(reportA).ok().unwrap_or(0);
            let value0 = a.value(&mut delay);

            if value0 {
                //button_pin.is_low().unwrap()
                led_pin.set_high().unwrap();
                //let _ = serial.write(b"A\n");
                keys[0] = Keyboard::A;
                //push_kb_movement(reportA).ok().unwrap_or(0);
            } else {
                led_pin.set_low().unwrap();
                keys[0] = Keyboard::NoEventIndicated;
            }
            let value1 = b.value(&mut delay);
            if value1 {
                led_pin.set_high().unwrap();
                //let _ = serial.write(b"A\n");
                keys[1] = Keyboard::D;
                //push_kb_movement(reportA).ok().unwrap_or(0);
            } else {
                led_pin.set_low().unwrap();
                keys[1] = Keyboard::NoEventIndicated;
            }
            count += 1.0;
            //     //let _ = serial.write(b"D\n");
            //     //push_kb_movement(reportD).ok().unwrap_or(0);
            // } else {
            //     led_pin.set_low().unwrap();
            // }
            // usb_dev.poll(&mut [&mut serial]);
            //     }
            //)
            //}

            // if input_count_down.wait().is_ok() {
            //let keys = Keyboard::NoEventIndicated;

            match keyboard.device().write_report(keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };
            // }
            if debug_count_down.wait().is_ok() {
                let endtime = timer.get_counter();
                let f = endtime - starttime;

                debug_rprintln!("Rate/s : {}", count * 1000000.0 / (f.to_micros() as f32));
            }

            if tick_count_down.wait().is_ok() {
                match keyboard.tick() {
                    Err(UsbHidError::WouldBlock) => {}
                    Ok(_) => {}
                    Err(e) => {
                        core::panic!("Failed to process keyboard tick: {:?}", e)
                    }
                };
            }

            if usb_dev.poll(&mut [&mut keyboard]) {
                match keyboard.device().read_report() {
                    Err(UsbError::WouldBlock) => {
                        //do nothing
                    }
                    Err(e) => {
                        core::panic!("Failed to read keyboard report: {:?}", e)
                    }
                    Ok(_leds) => {}
                }
            }
            // if button_pin.is_high().unwrap() {
            //     led_pin.set_high().unwrap();
            //     //let _ = serial.write(b"A\n");
            // } else {
            //     led_pin.set_low().unwrap();
            // }
        }
    }

    // fn core1_task() -> ! {
    //     loop {}
    // }
}
