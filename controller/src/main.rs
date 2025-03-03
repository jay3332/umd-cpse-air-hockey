//! Receives all commands from the serial port (USART1) and sends them to the peripheral devices.

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(let_chains)]

mod clock;
mod stepper;

use stepper::Stepper;

use arduino_hal::{
    hal::{
        port::*,
        usart::{BaudrateArduinoExt, Usart0},
        Pins as SrcPins,
    },
    prelude::*,
    DefaultClock, Peripherals, Pins,
};
use avr_device::atmega328p::TC0;
use embedded_hal::digital::OutputPin;
#[allow(unused_imports)]
use panic_halt;

/// Bitrate for the serial pin.
/// Current value taken from <https://github.com/jjrobots/Air_Hockey_Robot_EVO/blob/master/Arduino/AHRobot_EVO/AHRobot_EVO.ino#L38>
const BITRATE: u32 = 115_200;

/// The range of the simulated motor speed.
const RANGE: u16 = 10_000;

type Serial = Usart0<DefaultClock>;

pub struct Hardware {
    pub serial: Serial,
    pub stepper_a: Stepper<PD4, PD7>,
    pub stepper_b: Stepper<PD3, PD6>,
    pub tc0: TC0,
}

impl Hardware {
    const X_BOUNDS: (i32, i32) = (-10_000, 10_000);
    const Y_BOUNDS: (i32, i32) = (-10_000, 10_000);

    const MOVE_TO_POS_RANGE: u32 = 40_000; // Map [0, 255] to [0, 40_000]

    /// Computes the relative velocity of the paddle (X, Y), where X and Y are in the range
    /// [-10^4, 10^4]. The speed is relative to the maximum speed of the motor.
    #[inline]
    fn cartesian_to_corexy(mut x: i16, mut y: i16) -> (i16, i16) {
        x = -x; // Invert the X direction (such that right is positive)

        // If x + y > 1, scale the values such that x + y = 1.
        let total = (x + y).unsigned_abs().max((x - y).unsigned_abs());
        if total > RANGE {
            let scale = RANGE as f32 / total as f32;
            x = (x as f32 * scale) as _;
            y = (y as f32 * scale) as _;
        }

        // Define the two stepper motors as A and B, turning in the X direction clockwise.
        // * If A = -X and B = X, the paddle moves kX in the +Y direction (where k is a constant)
        // * If A = B = X, the paddle moves kX in the -X direction
        //
        // The speed of the motors is defined as:
        // * A = (X + Y) * MAX_SPEED
        // * B = (X - Y) * MAX_SPEED
        //
        // Reference: <http://wiki.fluidnc.com/en/config/kinematics>
        (x + y, x - y)
    }

    #[inline]
    const fn corexy_to_cartesian(a: i32, b: i32) -> (i32, i32) {
        (a + b, a - b)
    }

    pub const fn position(&self) -> (i32, i32) {
        Self::corexy_to_cartesian(self.stepper_a.steps, self.stepper_b.steps)
    }

    /// Runs the `V` command to run the paddle at a specified relative velocity.
    ///
    /// # Parameters
    /// - `x`: The relative velocity of the paddle in the X direction in the range [-10^4, 10^4].
    /// - `y`: The relative velocity the paddle in the Y direction in the range [-10^4, 10^4].
    pub fn run_at_rel_velocity(&mut self, vx: i16, vy: i16) {
        // Should we limit (x, y) based off the bounds?

        // let (x, y) = self.position();
        // if x <= Self::X_BOUNDS.0 && vx < 0
        //     || x >= Self::X_BOUNDS.1 && vx > 0
        //
        // {
        //     return;
        // }

        let (a, b) = Self::cartesian_to_corexy(vx, vy);

        self.stepper_a.run_at_speed(a);
        self.stepper_b.run_at_speed(b);
    }

    /// Moves the paddle to some specific position (x, y).
    ///
    /// # Parameters
    /// - `x`: The x-position of the puck to move to in the range [-32768, 32767].
    /// - `y`: The y-position of the puck to move to in the range [-32768, 32767].
    pub fn move_to_postion(&mut self, x: i16, y: i16) {
        // Normalize (x, y) to be in range
        let x = x as f32 / i16::MAX as f32 * Self::MOVE_TO_POS_RANGE as f32;
        let y = y as f32 / i16::MAX as f32 * Self::MOVE_TO_POS_RANGE as f32;
        let (a, b) = (x + y, x - y);

        self.stepper_a.move_to(a as i32 / 2, 10_000);
        self.stepper_b.move_to(b as i32 / 2, 10_000);
    }

    /// Returns the number of microseconds that have passed since the program started, with
    /// 4 µs resolution.
    #[inline]
    pub fn micros(&self) -> u32 {
        clock::micros(&self.tc0)
    }

    fn tick(&mut self) {
        let m = self.micros();
        let a = self.stepper_a.poll(m);
        let b = self.stepper_b.poll(m);

        if a > 1000 || b > 1000 {
            ufmt::uwriteln!(self.serial, "polled late: del_a={}, del_b={}", a, b).ok();
        }
    }
}

/// Receives ONLY 2-bit V commands.
pub fn start_controller_recv(hw: &mut Hardware) -> ! {
    let mut prev = None;
    loop {
        if let Ok(byte) = hw.serial.read() {
            if byte == 0xff {
                ufmt::uwriteln!(
                    hw.serial,
                    "DEBUG: {} mcs, a.steps={}, b.steps={}, pos={:?}",
                    hw.micros(),
                    hw.stepper_a.steps,
                    hw.stepper_b.steps,
                    hw.position(),
                )
                .ok();
            }
            let adjusted = byte as i8 as i16 * 100;
            if let Some(x) = prev.take() {
                hw.run_at_rel_velocity(x, adjusted);
            } else {
                prev = Some(adjusted);
            }
        }
        hw.tick();
    }
}

/// Receives ONLY 2-bit X commands.
// pub fn start_target_pos_recv(hw: &mut Hardware) -> ! {
//     let mut prev = None;
//     loop {
//         if let Ok(byte) = hw.serial.read() {
//             let adjusted = byte as i8 as i16 * 100;
//             if let Some(x) = prev.take() {
//                 hw.run_at_rel_velocity(x, adjusted);
//             } else {
//                 prev = Some(adjusted);
//             }
//         }
//         hw.tick();
//     }
// }

/// Receives all commands through the serial port.
///
/// # Serial Commands
///
/// Note: All integers are in little-endian format.
///
/// - ``V[X:i8][Y:i8]`` -> Set the velocity (relative speed) of the paddle to <X/10^4, Y/10^4>.
///   Constraints: -10^4 <= X <= 10^4 and -10^4 <= Y <= 10^4.
pub fn start_recv(hw: &mut Hardware) -> ! {
    #[derive(Copy, Clone, Debug, PartialEq)]
    enum Command {
        V,
        D,
    }

    impl Command {
        const MAX_BUFFER_SIZE: usize = 2;

        #[inline]
        const fn from_u8(byte: u8) -> Option<Self> {
            match byte {
                b'V' => Some(Self::V),
                b'D' => Some(Self::D),
                _ => None,
            }
        }

        #[inline]
        const fn bytes_needed(&self) -> usize {
            match self {
                Command::V => 2,
                Command::D => 0,
            }
        }

        fn done(&self, hw: &mut Hardware, bytes: &[u8]) {
            match self {
                Command::V => {
                    let x = bytes[0] as i8 as i16 * 100;
                    let y = bytes[1] as i8 as i16 * 100;
                    hw.run_at_rel_velocity(x, y);
                }
                Command::D => {
                    ufmt::uwriteln!(
                        hw.serial,
                        "DEBUG: {} mcs, a.steps={}, b.steps={}",
                        hw.micros(),
                        hw.stepper_a.steps,
                        hw.stepper_b.steps
                    )
                    .ok();
                }
            }
        }
    }

    let mut current = None::<Command>;
    let mut buffer = [0_u8; Command::MAX_BUFFER_SIZE];
    let mut buffer_idx = 0;

    loop {
        if let Some(cmd) = current
            && buffer_idx == cmd.bytes_needed()
        {
            cmd.done(hw, &buffer);
            current = None;
            buffer_idx = 0;
        }

        if let Ok(byte) = hw.serial.read() {
            if current.is_some() {
                buffer[buffer_idx] = byte;
                buffer_idx += 1;
            } else {
                current = Command::from_u8(byte);
            }
        }

        hw.tick();
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().expect("no arduino device?");
    let pins = Pins::with_mcu_pins(SrcPins::new(dp.PORTB, dp.PORTC, dp.PORTD));
    let mut serial = arduino_hal::Usart::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(),
        BaudrateArduinoExt::into_baudrate(BITRATE),
    );

    let stepper_a = Stepper::from_pins(pins.d4.into_output(), pins.d7.into_output());
    let stepper_b = Stepper::from_pins(pins.d3.into_output(), pins.d6.into_output());

    // Enable interrupts globally
    unsafe {
        // SAFETY: Not inside a critical section and any non-atomic operations have been completed
        // at this point.
        avr_device::interrupt::enable();
    }
    clock::prepare(&dp.TC0);

    ufmt::uwriteln!(serial, "Ready!").ok();
    let mut hw = Hardware {
        serial,
        stepper_a,
        stepper_b,
        tc0: dp.TC0,
    };
    start_controller_recv(&mut hw)
}
