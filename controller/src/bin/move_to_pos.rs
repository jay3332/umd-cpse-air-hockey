#![no_std]
#![no_main]

use arduino_hal::{
    hal::usart::{BaudrateArduinoExt, Usart0},
    prelude::*,
    DefaultClock, Pins,
};
use avr_device::asm::delay_cycles;
use avr_device::atmega328p::Peripherals;
use core::cmp::Ordering;
use embedded_hal::digital::OutputPin;
use micromath::F32Ext;
#[allow(unused_imports)]
use panic_halt;

const STEPS_PER_MM: u32 = 50; // Adjust for motor resolution
const MAX_SPEED: u16 = 30_000; // mm/s
const MAX_DELAY: u32 = 20_000; // µs, this is so the loop time doesn't suffer during acceleration

const ATTACK_CYCLES: u32 = 2_400_000; // The number of ticks to persist an attack for
const ATTACK_DISTANCE: i32 = 10; // The distance (mm) to move forward in a simple attack

// This represents one side of the symmetrical trapezoidal curve
const ACCEL_PROPORTION: f32 = 0.25; // max. proportion of a path spent accelerating
const MAX_ACCEL_DIST: f32 = 60.0; // max. distance, in mm, spent accelerating
const MAX_RAW_ACCEL: f32 = 20.0; // max. raw acceleration, in step/(4*delay^2)

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

    let mut step_a = pins.d2.into_output();
    let mut dir_a = pins.d5.into_output();
    let mut step_b = pins.d3.into_output();
    let mut dir_b = pins.d6.into_output();

    let mut current_a = 0;
    let mut current_b = 0;
    let mut current_va = 0_i32;
    let mut current_vb = 0_i32;

    let mut target = (0, 0, 0);
    let mut target_a = 0;
    let mut target_b = 0;
    let mut target_va = 0_i32;
    let mut target_vb = 0_i32;

    // let mut total_dist = 0_f32;
    // let mut accel_per_tick = 0_f32;

    let mut _dir_a_high = 1;
    let mut _dir_b_high = 1;

    let mut delay_a = MAX_DELAY;
    let mut delay_b = MAX_DELAY;

    let mut _a_done_accel = false;
    let mut _b_done_accel = false;

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

    macro_rules! update_vdir {
        () => {{
            _a_done_accel = false;
            _b_done_accel = false;
            if current_a > target_a {
                target_va *= -1;
            }
            if current_b > target_b {
                target_vb *= -1;
            }
        }};
    }

    // loop {
    //     let b = serial.read_byte();
    //     ufmt::uwriteln!(serial, "test {}", b).ok();
    // }

    for n in 0.. {
        if let Ok(cmd) = serial.read() {
            match Command::read(&mut serial, cmd) {
                Command::MoveToPos(x, y, v) => {
                    // if (x, y, v) != target {
                    // new target?
                    // ufmt::uwriteln!(serial, "target updated:  (x={} y={} v={})", x, y, v).ok();
                    target = (x, y, v);
                    target_a = (x as i32 - y as i32) * STEPS_PER_MM as i32;
                    target_b = (x as i32 + y as i32) * STEPS_PER_MM as i32;
                    target_va = v as i32 * STEPS_PER_MM as i32;
                    target_vb = v as i32 * STEPS_PER_MM as i32;

                    // current_v = target_v as f32;

                    // total_dist =
                    //     ((target_a - current_a) as f32).hypot((target_b - current_b) as f32);
                    // let accel_dist = (total_dist * ACCEL_PROPORTION)
                    //     .min(MAX_ACCEL_DIST * STEPS_PER_MM as f32);
                    // accel_per_tick = (target_v as f32 / accel_dist).min(MAX_RAW_ACCEL);

                    // // Normalize velocity vector
                    // let (current_x, current_y) = xy(current_a, current_b);
                    // let dx = x.abs_diff(current_x) as f32;
                    // let dy = y.abs_diff(current_y) as f32;
                    //
                    // let (ky, kx) = (dy / dx).atan().sin_cos();
                    // let v = v as f32 * STEPS_PER_MM as f32;
                    // let vx = (v * kx) as i32;
                    // let vy = (v * ky) as i32;
                    //
                    // target_v = (vx - vy, vx + vy);

                    update_vdir!();
                    // }
                }
                Command::Status => {
                    // send 8 bytes: B[current_x:i16][current_y:i16][is_busy:bool][checksum]\n
                    let (x, y) = xy(current_a, current_b);
                    let is_busy = (current_a, current_b) != (target_a, target_b);

                    // checksum = (x + y) % 256
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
                    _a_done_accel = false;
                    _b_done_accel = false;
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

                    update_vdir!();
                }
                Command::None => (),
            }
        }

        // let progress = if total_dist == 0.0 {
        //     1.0
        // } else {
        //     let dist = ((target_a - current_a) as f32).hypot((target_b - current_b) as f32);
        //     dist / total_dist
        // };
        //
        // if (target_v as f32 - current_v).abs() > accel_per_tick + f32::EPSILON {
        //     current_v += accel_per_tick;
        // }

        const ACCEL: i32 = 300;
        const CYCLES_PER_SEC: u32 = 16_000_000;

        if !_a_done_accel {
            match current_va.cmp(&target_va) {
                Ordering::Less => {
                    current_va = (current_va + ACCEL).min(target_va);
                    if current_va == 0 {
                        if target_va == 0 {
                            _a_done_accel = true;
                        }
                    } else {
                        delay_a = (CYCLES_PER_SEC / current_va.unsigned_abs()).min(MAX_DELAY);
                    }
                }
                Ordering::Greater => {
                    current_va = (current_va - ACCEL).max(target_va);
                    if current_va == 0 {
                        if target_va == 0 {
                            _a_done_accel = true;
                        }
                    } else {
                        delay_a = (CYCLES_PER_SEC / current_va.unsigned_abs()).min(MAX_DELAY);
                    }
                }
                _ => {
                    _a_done_accel = true;
                }
            }
        }
        if !_b_done_accel {
            match current_vb.cmp(&target_vb) {
                Ordering::Less => {
                    current_vb = (current_vb + ACCEL).min(target_vb);
                    if current_vb == 0 {
                        if target_vb == 0 {
                            _b_done_accel = true;
                        }
                    } else {
                        delay_b = (CYCLES_PER_SEC / current_vb.unsigned_abs()).min(MAX_DELAY);
                    }
                }
                Ordering::Greater => {
                    current_vb = (current_vb - ACCEL).max(target_vb);
                    if current_vb == 0 {
                        if target_vb == 0 {
                            _b_done_accel = true;
                        }
                    } else {
                        delay_b = (CYCLES_PER_SEC / current_vb.unsigned_abs()).min(MAX_DELAY);
                    }
                }
                _ => {
                    _b_done_accel = true;
                }
            }
        }

        // if n % 20000 == 0 {
        //     ufmt::uwriteln!(
        //         serial,
        //         "current: ({}, {}), target: ({}, {}) cv: ({}, {}), tv: ({}, {}), delay: ({}, {}), dir: ({}, {})",
        //         current_a,
        //         current_b,
        //         target_a,
        //         target_b,
        //         current_va,
        //         current_vb,
        //         target_va,
        //         target_vb,
        //         delay_a,
        //         delay_b,
        //         dir_a.is_set_high(),
        //         dir_b.is_set_high(),
        //     )
        //     .ok();
        // }

        if current_va > 0 && _dir_a_high == -1 {
            dir_a.set_high();
            _dir_a_high = 1;
        }
        if current_va < 0 && _dir_a_high == 1 {
            dir_a.set_low();
            _dir_a_high = -1;
        }
        if current_vb > 0 && _dir_b_high == -1 {
            dir_b.set_high();
            _dir_b_high = 1;
        }
        if current_vb < 0 && _dir_b_high == 1 {
            dir_b.set_low();
            _dir_b_high = -1;
        }

        if current_a == target_a {
            current_va = 0;
        }
        if current_b == target_b {
            current_vb = 0;
        }

        let a_needs_step = current_va != 0 && current_a != target_a;
        let b_needs_step = current_vb != 0 && current_b != target_b;

        if attacking {
            attack_ticks_remaining =
                attack_ticks_remaining.saturating_sub(if !a_needs_step && !b_needs_step {
                    MAX_DELAY
                } else if delay_a == delay_b {
                    delay_a * 2
                } else {
                    delay_a * 2 + delay_b * 2
                });

            if attack_ticks_remaining == 0 {
                ufmt::uwriteln!(serial, "Attack done!").ok();
                attacking = false;
                target_a = attack_return_state.0;
                target_va = attack_return_state.1;
                target_b = attack_return_state.2;
                target_vb = attack_return_state.3;
                update_vdir!();
            }
        }
        if !a_needs_step && !b_needs_step {
            delay_cycles(MAX_DELAY);
            continue;
        }

        if delay_a == delay_b {
            if a_needs_step {
                step_a.set_high();
            }
            if b_needs_step {
                step_b.set_high();
            }
            delay_cycles(delay_a);
            if a_needs_step {
                step_a.set_low();
                current_a += _dir_a_high;
            }
            if b_needs_step {
                step_b.set_low();
                current_b += _dir_b_high;
            }
            delay_cycles(delay_a);
        } else {
            if a_needs_step {
                step_a.set_high();
                delay_cycles(delay_a);
                step_a.set_low();
                current_a += _dir_a_high;
                delay_cycles(delay_a);
            }
            if b_needs_step {
                step_b.set_high();
                delay_cycles(delay_b);
                step_b.set_low();
                current_b += _dir_b_high;
                delay_cycles(delay_b);
            }
        }
    }
    panic!("wtf")
}

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
