//! A background clock that ticks every 4 microseconds.

use arduino_hal::{clock::Clock, pac::TC0};
use avr_device::interrupt::Mutex;
use core::cell::Cell;

/// The clock frequency of the ATmega328P, in kHz.
pub const FREQ_KHZ: u32 = arduino_hal::DefaultClock::FREQ / 1_000; // 16_000 kHz

/// The prescaled clock frequency of the ATmega328P, in kHz.
pub const PRESCALED_FREQ_KHZ: u32 = FREQ_KHZ / 64; // 250 kHz

/// The number of prescaled clock ticks for each overflow.
pub const TICKS_PER_OVERFLOW: u8 = 250;

/// The number of milliseconds it will take to reach an overflow and thus an interrupt.
pub const MILLIS_PER_OVERFLOW: u32 = TICKS_PER_OVERFLOW as u32 / PRESCALED_FREQ_KHZ; // 1 ms

/// The number of microseconds a prescaled tick will take.
pub const MICROS_PER_TICK: u32 = 1_000 / PRESCALED_FREQ_KHZ; // 4 µs

static OVERFLOWS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Prepare the timer.
pub fn prepare(tc0: &TC0) {
    // Configure the timer for the above interval (in CTC mode) and enable its interrupt.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(TICKS_PER_OVERFLOW));
    tc0.tccr0b.write(|w| w.cs0().prescale_64()); // prescaler: /64
    tc0.timsk0.write(|w| w.ocie0a().set_bit()); // enable interrupts

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        OVERFLOWS.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = OVERFLOWS.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + 1);
    })
}

/// Returns the number of milliseconds that have occurred since the program started.
pub fn millis() -> u32 {
    avr_device::interrupt::free(|cs| OVERFLOWS.borrow(cs).get()) * MILLIS_PER_OVERFLOW
}

/// Returns the number of microseconds that have passed since the program started. (4 µs resolution)
///
/// Requires the `TC0` timer for precision (checks the cycle count).
pub fn micros(tc0: &TC0) -> u32 {
    let (mut overflows, ticks, tifr) = avr_device::interrupt::free(|cs| {
        let overflows = OVERFLOWS.borrow(cs).get();

        // The number of ticks that have occurred since the last overflow.
        let ticks = tc0.tcnt0.read().bits();
        // Has the timer overflowed since the last read?
        let tifr = tc0.tifr0.read().ocf0a().bit();

        (overflows, ticks, tifr)
    });

    // Account for the overflow interrupt that may have occurred
    if tifr && ticks < TICKS_PER_OVERFLOW {
        overflows += 1;
    }

    overflows * MILLIS_PER_OVERFLOW * 1_000 + ticks as u32 * MICROS_PER_TICK
}
