#![no_std]
#![no_main]

use arduino_hal::{
    delay_us,
    hal::usart::{BaudrateArduinoExt, Usart0},
    prelude::*,
    DefaultClock, Pins,
};
use avr_device::asm::delay_cycles;
use avr_device::atmega328p::Peripherals;
#[allow(unused_imports)]
use panic_halt;

const STEPS_PER_MM: u32 = 50; // Adjust for motor resolution

const BITRATE: u32 = 115_200;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().expect("no arduino device?");
    let pins = Pins::with_mcu_pins(arduino_hal::hal::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD));
    let mut serial = arduino_hal::Usart::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(),
        BaudrateArduinoExt::into_baudrate(BITRATE),
    );

    let mut step = pins.d3.into_output();
    let mut dir = pins.d6.into_output();

    let mut current = 0;
    let mut current_v = 50;

    let mut target = 200 * STEPS_PER_MM as i32;
    let mut target_v = 16000 * STEPS_PER_MM as i32;

    #[inline]
    fn xy(a: i32, b: i32) -> (i16, i16) {
        let x = (a + b) as i16 / 2;
        let y = (b - a) as i16 / 2;
        (x, y)
    }

    ufmt::uwriteln!(serial, "Ready!").ok();

    //400
    loop {
        if (current_v < target_v) {
            current_v += 1000;
        }
        // if current_v > target_v {
        //     current_v = 50;
        // }

        // Convert speed to delay in µs
        let delay = (16_000_000 / current_v as u32);

        step.set_high();
        delay_cycles(delay);
        step.set_low();
        delay_cycles(delay);
    }

    panic!("wtf")
}
