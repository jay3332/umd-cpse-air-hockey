//! Receives all commands from the serial port (USART1) and sends them to the peripheral devices.

#![no_std]
#![no_main]

mod stepper;
use stepper::Stepper;

use arduino_hal::{
    hal::{
        usart::{BaudrateArduinoExt, Usart0},
        Pins as SrcPins,
    },
    prelude::*,
    DefaultClock, Peripherals, Pins,
};
#[allow(unused_imports)]
use panic_halt;

/// Bitrate for the serial pin.
/// Current value taken from <https://github.com/jjrobots/Air_Hockey_Robot_EVO/blob/master/Arduino/AHRobot_EVO/AHRobot_EVO.ino#L38>
const BITRATE: u32 = 115_200;

type Serial = Usart0<DefaultClock>;

pub struct Hardware {
    pub serial: Serial,
    pub stepper_a: Stepper,
    pub stepper_b: Stepper,
}

impl Hardware {
    /// Maximum motor speed, in revolutions per minute.
    const MOTOR_MAX_SPEED: i32 = 32_000;

    /// Computes the relative velocity of the paddle (X, Y), where X and Y are in the range [-1, 1].
    /// The speed is relative to the maximum speed of the motor.
    #[inline]
    const fn corexy_to_cartesian(mut x: f32, mut y: f32) -> (f32, f32) {
        // If x + y > 1, scale the values such that x + y = 1.
        if x + y > 1.0 {
            let scale = 1.0 / (x + y);
            x *= scale;
            y *= scale;
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
        let a = (x + y) * Self::MOTOR_MAX_SPEED as f32;
        let b = (x - y) * Self::MOTOR_MAX_SPEED as f32;
        (a, b)
    }

    /// Runs the `V` command to set the relative speed of the paddle.
    ///
    /// # Parameters
    /// - `x`: The relative speed of the paddle in the X direction in the range [-10^4, 10^4].
    /// - `y`: The relative speed of the paddle in the Y direction in the range [-10^4, 10^4].
    pub fn set_rel_speed(&mut self, x: i16, y: i16) {
        const RANGE: i16 = 10_000;
        let (a, b) = Self::corexy_to_cartesian(x as f32 / RANGE as f32, y as f32 / RANGE as f32);

        ufmt::uwriteln!(
            self.serial,
            "setting relative motor speeds a={}, b={}",
            x,
            y
        )
        .ok();
        self.stepper_a.set_speed(a);
        self.stepper_b.set_speed(b);
    }

    fn tick(&mut self) {
        self.stepper_a.step();
        self.stepper_b.step();
    }
}

/// Receives all commands through the serial port.
///
/// # Serial Commands
///
/// Note: All integers are in little-endian format.
///
/// - ``V[X:i16][Y:i16]`` -> Set the velocity (relative speed) of the paddle to <X/10^4, Y/10^4>.
///   Constraints: -10^4 <= X <= 10^4 and -10^4 <= Y <= 10^4.
pub fn start_recv(hw: &mut Hardware) -> ! {
    #[derive(Copy, Clone, Debug, PartialEq)]
    enum Command {
        V,
    }

    impl Command {
        const MAX_BUFFER_SIZE: usize = 4;

        #[inline]
        const fn from_u8(byte: u8) -> Option<Self> {
            match byte {
                b'V' => Some(Self::V),
                _ => None,
            }
        }

        #[inline]
        const fn bytes_needed(&self) -> usize {
            match self {
                Command::V => 4,
            }
        }

        fn done(&self, hw: &mut Hardware, bytes: &[u8]) {
            match self {
                Command::V => {
                    let x = i16::from_le_bytes([bytes[0], bytes[1]]);
                    let y = i16::from_le_bytes([bytes[2], bytes[3]]);
                    hw.set_rel_speed(x, y);
                }
            }
        }
    }

    let mut current = None::<Command>;
    let mut buffer = [0_u8; Command::MAX_BUFFER_SIZE];
    let mut buffer_idx = 0;

    loop {
        if let Ok(byte) = hw.serial.read() {
            if let Some(cmd) = current {
                buffer[buffer_idx] = byte;
                buffer_idx += 1;
                if buffer_idx == cmd.bytes_needed() {
                    cmd.done(hw, &buffer);
                    current = None;
                    buffer_idx = 0;
                }
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
    ufmt::uwriteln!(serial, "Ready!").ok();

    let stepper_a = Stepper::from_pins(
        pins.d2.into_output().downgrade(),
        pins.d5.into_output().downgrade(),
    );
    let stepper_b = Stepper::from_pins(
        pins.d3.into_output().downgrade(),
        pins.d6.into_output().downgrade(),
    );

    let mut hw = Hardware {
        serial,
        stepper_a,
        stepper_b,
    };
    start_recv(&mut hw)
}
