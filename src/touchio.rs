#![feature(trait_alias)]
const N_SAMPLES: i32 = 10;
const TIMEOUT_TICKS: u16 = 10000;
use core::{
    any::Any,
    borrow::{Borrow, BorrowMut},
    ops::{Deref, DerefMut},
};

use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::ExtU64;

use rp2040_hal::gpio::{Function, Pin, PinId, PullType};
use rtic_monotonics::rp2040::Timer;
impl<
        A: PinId
            + rp2040_hal::gpio::ValidFunction<rp2040_hal::gpio::FunctionNull>
            + rp2040_hal::gpio::ValidFunction<
                rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioOutput>,
            > + rp2040_hal::gpio::ValidFunction<
                rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioInput>,
            >,
        F: Function,
        P: PullType,
    > Touchio<A, F, P>
{
    pub async fn get_raw_reading(mut self) -> u16 {
        let mut ticks: u16 = 0;
        // let pin: &mut Pin<DynPinId, FunctionNull, PullDown> =
        //     &mut self.pin.try_into_function().ok().unwrap();

        //let mut pin =
        let mut pin = self.pin.into_pull_down_disabled();

        for _ in 0..N_SAMPLES {
            let mut a = pin.into_push_pull_output();
            a.set_high().unwrap();
            Timer::delay(10_u64.micros()).await;

            let b = a.into_floating_input();
            while b.is_high().unwrap() {
                if ticks >= TIMEOUT_TICKS {
                    return TIMEOUT_TICKS;
                }
                ticks += 1;
            }
            pin = b.into_pull_down_disabled();
        }
        ticks
    }
    pub fn new<'a>(pin: P) -> Touchio<A, F, P> {
        let mut a = Touchio { pin, threshold: 0 };
        let mut raw_reading = a.get_raw_reading();
        if raw_reading == TIMEOUT_TICKS {
            //panic!("No pulldown on pin; 1Mohm recommended");
            raw_reading = 1000;
        }
        let threshold = ((raw_reading as f32) * 1.05 + 100.0) as u16;

        Touchio { pin, threshold }
    }
    pub fn value(&mut self) -> bool {
        let reading = self.get_raw_reading();
        reading > self.threshold
    }
}

pub struct Touchio<A, F, P>
where
    A: PinId,
    F: Function,
    P: PullType,
{
    pin: Pin<A, F, P>,
    threshold: u16,
}
