#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::{
    hal::usart::{BaudrateArduinoExt, Usart0},
    prelude::*,
    DefaultClock, Pins,
};
use avr_device::{
    atmega328p::{tc1::tccr1b::CS1_A, TC1, TC2, USART0},
    interrupt,
};
use core::{
    cmp::Ordering,
    sync::atomic::{AtomicBool, AtomicIsize as AtomicI32, Ordering::Relaxed},
};
use avr_device::atmega328p::Peripherals;
use embedded_hal::digital::OutputPin;
use micromath::F32Ext;
#[allow(unused_imports)]
use panic_halt;

const STEPS_PER_MM: u32 = 50; // Adjust for motor resolution
const MAX_SPEED: u16 = 30_000; // mm/s
const MAX_DELAY: u32 = 20_000; // µs, this is so the loop time doesn't suffer during acceleration

const ATTACK_CYCLES: u32 = 2_400_000; // The number of ticks to persist an attack for
const ATTACK_DISTANCE: i32 = 10; // The distance (mm) to move forward in a simple attack

const BITRATE: u32 = 115_200;

// Constants from C code
const ZERO_SPEED: u16 = 65535;
const SCURVE_LOW_SPEED: i16 = 1000;
const MIN_ACCEL: i16 = 100;

// Global variables for interrupt handlers
static POSITION_A: AtomicI32 = AtomicI32::new(0);
static POSITION_B: AtomicI32 = AtomicI32::new(0);
static DIR_A: AtomicI32 = AtomicI32::new(0);
static DIR_B: AtomicI32 = AtomicI32::new(0);
static SPEED_A: AtomicI32 = AtomicI32::new(0);
static SPEED_B: AtomicI32 = AtomicI32::new(0);
static STEP_A_PENDING: AtomicBool = AtomicBool::new(false);
static STEP_B_PENDING: AtomicBool = AtomicBool::new(false);

enum Command {
    MoveToPos(i16, i16, u16),
    Status,
    Reset,
    SimpleAttack,
    None,
}

impl Command {
    fn read_move_to_pos(serial: &mut Usart0<DefaultClock>) -> Self {
        let x = read_i16(serial);
        let y = read_i16(serial);
        let v = read_u16(serial).clamp(0, MAX_SPEED);

        // checksum = (x + y + v) % 256
        let checksum = x.wrapping_add(y).wrapping_add(v as i16).rem_euclid(256);
        if serial.read_byte() as i16 != checksum {
            ufmt::uwriteln!(serial, "({} {} {}) bad checksum!", x, y, v).ok();
            return Self::None;
        }

        Self::MoveToPos(x, y, v)
    }

    fn read(serial: &mut Usart0<DefaultClock>, hdr: u8) -> Self {
        // ufmt::uwriteln!(serial, "reading cmd {}", hdr).ok();
        match hdr {
            b'M' => Self::read_move_to_pos(serial),
            b'B' => Self::Status,
            b'R' => Self::Reset,
            b'A' => Self::SimpleAttack,
            _ => Self::None,
        }
    }
}

fn read_u16(serial: &mut Usart0<DefaultClock>) -> u16 {
    u16::from_be_bytes([serial.read_byte(), serial.read_byte()])
}

fn read_i16(serial: &mut Usart0<DefaultClock>) -> i16 {
    i16::from_be_bytes([serial.read_byte(), serial.read_byte()])
}

#[arduino_hal::entry]
fn main() -> ! {
    let mut dp = Peripherals::take().expect("no arduino device?");
    let pins = Pins::with_mcu_pins(arduino_hal::hal::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD));
    let mut serial = arduino_hal::Usart::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(),
        BaudrateArduinoExt::into_baudrate(BITRATE),
    );

    let mut step_a = pins.d2.into_output();
    let mut dir_a = pins.d5.into_output();
    let mut step_b = pins.d3.into_output();
    let mut dir_b = pins.d6.into_output();

    // Initialize timer interrupts
    setup_timer_interrupts(&mut dp.TC1, &mut dp.TC2);

    // Enable interrupts
    unsafe { avr_device::interrupt::enable() };

    let mut current_a = 0;
    let mut current_b = 0;
    let mut current_va = 0_i32;
    let mut current_vb = 0_i32;

    let mut target = (0, 0, 0);
    let mut target_a = 0;
    let mut target_b = 0;
    let mut target_va = 0_i32;
    let mut target_vb = 0_i32;

    let mut max_acceleration = 2000; // steps/s^2
    let mut user_max_speed = MAX_SPEED as i32 * STEPS_PER_MM as i32;

    let mut attacking = false;
    let mut attack_ticks_remaining = 0;
    let mut attack_return_state = (0, 0, 0, 0); // target a, va, b, va


    #[inline]
    fn xy(a: i32, b: i32) -> (i16, i16) {
        let x = (a + b) as i16 / 2;
        let y = (b - a) as i16 / 2;
        (x, y)
    }

    ufmt::uwriteln!(serial, "Ready!").ok();

    for n in 0.. {
        if let Ok(cmd) = serial.read() {
            match Command::read(&mut serial, cmd) {
                Command::MoveToPos(x, y, v) => {
                    target = (x, y, v);
                    target_a = (x as i32 - y as i32) * STEPS_PER_MM as i32;
                    target_b = (x as i32 + y as i32) * STEPS_PER_MM as i32;
                    target_va = v as i32 * STEPS_PER_MM as i32;
                    target_vb = v as i32 * STEPS_PER_MM as i32;

                    // Update global variables for interrupts
                    DIR_A.store(if target_a > current_a { 1 } else if target_a == current_a {0} else { -1 }, Relaxed);
                    DIR_B.store(if target_b > current_b { 1 } else if target_b == current_b {0} else { -1 }, Relaxed);
                }
                Command::Status => {
                    let (x, y) = xy(current_a, current_b);
                    let is_busy = (current_a, current_b) != (target_a, target_b);
                    let checksum = (x.wrapping_add(y) % 256) as u8;
                    let [x1, x2] = x.to_be_bytes();
                    let [y1, y2] = y.to_be_bytes();

                    [b'B', x1, x2, y1, y2, is_busy as u8, checksum, b'\n']
                        .into_iter()
                        .for_each(|b| {
                            serial.write(b).ok();
                        });
                    serial.flush();
                }
                Command::Reset => {
                    current_a = 0;
                    current_b = 0;
                    current_va = 0;
                    current_vb = 0;
                    target = (0, 0, 0);
                    target_a = 0;
                    target_b = 0;
                    target_va = 0;
                    target_vb = 0;
                    DIR_A.store(0, Relaxed);
                    DIR_B.store(0, Relaxed);
                }
                Command::SimpleAttack => {
                    ufmt::uwriteln!(serial, "Simple attack!").ok();
                    attacking = true;
                    attack_return_state = (target_a, target_va, target_b, target_vb);
                    attack_ticks_remaining = ATTACK_CYCLES;

                    let (x, y) = xy(current_a, current_b);
                    let attack_y = y - ATTACK_DISTANCE as i16 * STEPS_PER_MM as i16;
                    target_a = (x - attack_y) as i32;
                    target_b = (x + attack_y) as i32;

                    const ATTACK_VELOCITY: i32 = 2200;
                    const V: i32 = ATTACK_VELOCITY * STEPS_PER_MM as i32;
                    target_va = V;
                    target_vb = V;

                    DIR_A.store(1, Relaxed);
                    DIR_B.store(1, Relaxed);
                }
                Command::None => (),
            }
        }

        // Update current positions from interrupt handlers
        current_a = POSITION_A.load(Relaxed) as i32;
        current_b = POSITION_B.load(Relaxed) as i32;

        // Position control logic
        position_control(
            current_a,
            current_b,
            target_a,
            target_b,
            target_va,
            target_vb,
            max_acceleration,
            user_max_speed,
            &mut dp.TC1,
            &mut dp.TC2,
        );

        if attacking {
            attack_ticks_remaining = attack_ticks_remaining.saturating_sub(1);
            if attack_ticks_remaining == 0 {
                ufmt::uwriteln!(serial, "Attack done!").ok();
                attacking = false;
                target_a = attack_return_state.0;
                target_va = attack_return_state.1;
                target_b = attack_return_state.2;
                target_vb = attack_return_state.3;
                DIR_A.store(if target_a > current_a { 1 } else { -1 }, Relaxed);
                DIR_B.store(if target_b > current_b { 1 } else { -1 }, Relaxed);
            }
        }

        // Handle step pulses from interrupts
        if STEP_A_PENDING.load(Relaxed) {
            step_a.set_high();
            arduino_hal::delay_us(1);
            step_a.set_low();
            STEP_A_PENDING.store(false, Relaxed);
        }
        if STEP_B_PENDING.load(Relaxed) {
            step_b.set_high();
            arduino_hal::delay_us(1);
            step_b.set_low();
            STEP_B_PENDING.store(false, Relaxed);
        }
    }
    panic!("wtf")
}

fn setup_timer_interrupts(tc1: &mut TC1, tc2: &mut TC2) {
    // Configure Timer1 for motor A
    tc1.tccr1a.write(|w| w.wgm1().bits(0b00)); // Normal mode
    tc1.tccr1b.write(|w| {
        w.cs1().variant(CS1_A::PRESCALE_8); // Prescaler 8
        w.wgm1().bits(0b00)
    });
    tc1.timsk1.write(|w| w.ocie1a().set_bit()); // Enable compare match interrupt

    // Configure Timer3 for motor B
    tc2.tccr2a.write(|w| w.wgm2().bits(0b00)); // Normal mode
    tc2.tccr2b.write(|w| {
        w.cs2().prescale_8(); // Prescaler 8
        w.wgm22().bit(false)
    });
    tc2.timsk2.write(|w| w.ocie2a().set_bit()); // Enable compare match interrupt
}

fn position_control(
    current_a: i32,
    current_b: i32,
    target_a: i32,
    target_b: i32,
    target_va: i32,
    target_vb: i32,
    max_acceleration: i32,
    user_max_speed: i32,
    tc1: &mut TC1,
    tc2: &mut TC2,
) {
    // Implement the position control algorithm from C code
    let speed_a = SPEED_A.load(Relaxed) as i32;
    let speed_b = SPEED_B.load(Relaxed) as i32;

    // Calculate stop positions
    let temp_a = (speed_a * speed_a) / (1900 * max_acceleration);
    let pos_stop_a = current_a + sign(speed_a) * temp_a;

    let temp_b = (speed_b * speed_b) / (1900 * max_acceleration);
    let pos_stop_b = current_b + sign(speed_b) * temp_b;

    // Determine target speeds
    let result_speed_a = if target_a > current_a {
        if pos_stop_a >= target_a { 0 } else { target_va }
    } else {
        if pos_stop_a <= target_a { 0 } else { -target_va }
    };

    let result_speed_b = if target_b > current_b {
        if pos_stop_b >= target_b { 0 } else { target_vb }
    } else {
        if pos_stop_b <= target_b { 0 } else { -target_vb }
    };

    // Update motor speeds
    set_motor_speed(result_speed_a, result_speed_b, max_acceleration, user_max_speed, tc1, tc2);
}

fn set_motor_speed(speed_a: i32, speed_b: i32, max_acceleration: i32, user_max_speed: i32, tc1: &mut TC1, tc2: &mut TC2) {
    // Constrain speeds
    let speed_a = speed_a.clamp(-user_max_speed, user_max_speed);
    let speed_b = speed_b.clamp(-user_max_speed, user_max_speed);

    // Update global speed variables
    SPEED_A.store(speed_a as isize, Relaxed);
    SPEED_B.store(speed_b as isize, Relaxed);

    // Calculate timer periods (2MHz timer)
    let period_a = if speed_a == 0 {
        ZERO_SPEED
    } else {
        (2_000_000 / speed_a.abs()).clamp(1, ZERO_SPEED as i32) as u16
    };

    let period_b = if speed_b == 0 {
        ZERO_SPEED
    } else {
        (2_000_000 / speed_b.abs()).clamp(1, ZERO_SPEED as i32) as u16
    };

    // Set timer compare values
    unsafe {
        tc1.ocr1a.write(|w| w.bits(period_a));
        tc2.ocr2a.write(|w| w.bits(period_b as u8));
    }
}

fn sign(value: i32) -> i32 {
    if value > 0 { 1 } else if value < 0 { -1 } else { 0 }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    let dir = DIR_A.load(Relaxed);
    if dir == 0 {
        return;
    }

    let pos = POSITION_A.load(Relaxed);
    POSITION_A.store(pos + dir, Relaxed);
    STEP_A_PENDING.store(true, Relaxed);
}

#[avr_device::interrupt(atmega328p)]
fn TIMER2_COMPA() {
    let dir = DIR_B.load(Relaxed);
    if dir == 0 {
        return;
    }

    let pos = POSITION_B.load(Relaxed);
    POSITION_B.store(pos + dir, Relaxed);
    STEP_B_PENDING.store(true, Relaxed);
}
