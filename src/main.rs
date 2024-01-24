#![no_std]
#![no_main]

const N_SAMPLES: i32 = 10;
const TIMEOUT_TICKS: u16 = 10000;

use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use usbd_human_interface_device::descriptor::InterfaceProtocol;
use usbd_human_interface_device::device::keyboard::{
    BootKeyboardConfig, BOOT_KEYBOARD_REPORT_DESCRIPTOR,
};
use usbd_human_interface_device::interface::{InterfaceBuilder, ManagedIdleInterfaceConfig};
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;
use vcc_gnd_yd_rp2040::hal::multicore::{Multicore, Stack};

use core::borrow::BorrowMut;
use cortex_m::delay::Delay;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use hal::Timer;
use panic_probe as _;
use rtt_target::{debug_rprintln, debug_rtt_init_print};
// use rp2040_hal as halpi;
use usb_device::{class_prelude::*, prelude::*};

use vcc_gnd_yd_rp2040::hal::gpio::DynPin;
use vcc_gnd_yd_rp2040::{entry, hal};
use vcc_gnd_yd_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
//static mut CORE1_STACK: Stack<4096> = Stack::new();
#[entry]
fn main() -> ! {
    debug_rtt_init_print!();
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
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

    let mut led_pin = pins.led.into_push_pull_output();
    let mut _button_pin = pins.user_key.into_pull_up_input();
    let mut binding_a = pins.gpio11.into()
    let mut binding_b = pins.gpio6.into();
    let mut a = Touchio::new(&mut binding_a, &mut delay);
    let mut b = Touchio::new(&mut binding_b, &mut delay);

    // loop {
    //     push_kb_movement(report).ok().unwrap_or(0);
    // }
    // let mut input_count_down = timer.count_down();
    // input_count_down.start(1.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    let mut debug_count_down = timer.count_down();
    debug_count_down.start(2000.millis());

    // let mut tick_osuclick = timer.count_down();
    // tick_osuclick.start(1.millis());
    let mut keys: [Keyboard; 2] = [Keyboard::NoEventIndicated; 2];

    // let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    // let cores = mc.cores();
    // let core1 = &mut cores[1];
    // let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);
    let mut count: f32 = 0.0;
    let mut starttime = timer.get_counter();
    loop {
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
        //     //push_kb_movement(reportA).ok().unwrap_or(0);
        // } else {
        //     led_pin.set_low().unwrap();
        // }
    }
}

// fn core1_task() -> ! {
//     loop {}
// }

impl Touchio<'_> {
    fn get_raw_reading(&mut self, delay: &mut Delay) -> u16 {
        let mut ticks: u16 = 0;
        let pin = self.pin.borrow_mut();
        for _ in 0..N_SAMPLES {
            pin.into_push_pull_output();
            let _ = pin.set_high();
            delay.delay_us(10);
            pin.into_floating_input();
            while pin.is_high().unwrap() {
                if ticks >= TIMEOUT_TICKS {
                    return TIMEOUT_TICKS;
                }
                ticks += 1;
            }
            pin.into_pull_down_disabled();
        }
        ticks
    }
    fn new<'a>(pin: &'a mut DynPin, delay: &mut Delay) -> Touchio<'a> {
        let mut a = Touchio { pin, threshold: 0 };
        let mut raw_reading = a.get_raw_reading(delay);
        if raw_reading == TIMEOUT_TICKS {
            //panic!("No pulldown on pin; 1Mohm recommended");
            raw_reading = 1000;
        }
        let threshold = ((raw_reading as f32) * 1.05 + 100.0) as u16;

        Touchio { pin, threshold }
    }
    fn value(&mut self, delay: &mut Delay) -> bool {
        let reading = self.get_raw_reading(delay);
        reading > self.threshold
    }
}

struct Touchio<'a> {
    pin: &'a mut DynPin,
    threshold: u16,
}
