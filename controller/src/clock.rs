//! A background clock that ticks every 4 microseconds.

use arduino_hal::{clock::Clock, pac::TC1};
use avr_device::interrupt::Mutex;
use core::cell::Cell;

/// The clock frequency of the ATmega328P, in MHz.
pub const FREQ_UHZ: u32 = arduino_hal::DefaultClock::FREQ / 1_000_000; // 16 MHz

/// The prescaled clock frequency of the ATmega328P, in MHz.
pub const PRESCALED_FREQ_UHZ: u32 = FREQ_UHZ / 8; // 2 MHz
/// The number of prescaled clock ticks for each overflow.
pub const TICKS_PER_OVERFLOW: u16 = 8;

/// The number of microseconds it will take to reach an overflow and thus an interrupt.
pub const MICROS_PER_OVERFLOW: u32 = TICKS_PER_OVERFLOW as u32 / PRESCALED_FREQ_UHZ; // 4 µs

static INTERRUPTS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Prepare the timer.
pub fn prepare(tc1: TC1) {
    // Configure the timer for the above interval (in CTC mode) and enable its interrupt.
    tc1.tccr1a.write(|w| w.wgm1().bits(0b0100)); // 0b0100 = mode 4 (UP to OCR1A)
    tc1.tccr1b.write(|w| w.cs1().prescale_8()); // prescaler: /8
    tc1.ocr1a.write(|w| w.bits(TICKS_PER_OVERFLOW));
    tc1.timsk1.write(|w| w.ocie1a().set_bit()); // enable interrupts

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        INTERRUPTS.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = INTERRUPTS.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + 1);
    })
}

/// Returns the number of timer interrupts that have occurred since the program started.
pub fn interrupts() -> u32 {
    avr_device::interrupt::free(|cs| INTERRUPTS.borrow(cs).get())
}

/// Returns the number of microseconds that have passed since the program started.
pub fn micros() -> u32 {
    interrupts() * MICROS_PER_OVERFLOW
}
