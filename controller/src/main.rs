//! Receives all commands from the serial port (USART1) and sends them to the peripheral devices.

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod clock;
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

/// The range of the simulated motor speed.
const RANGE: u16 = 10_000;

type Serial = Usart0<DefaultClock>;

pub struct Hardware {
    pub serial: Serial,
    pub stepper_a: Stepper,
    pub stepper_b: Stepper,
}

impl Hardware {
    /// Computes the relative velocity of the paddle (X, Y), where X and Y are in the range
    /// [-10^4, 10^4]. The speed is relative to the maximum speed of the motor.
    #[inline]
    const fn corexy_to_cartesian(mut x: i16, mut y: i16) -> (i16, i16) {
        // If x + y > 1, scale the values such that x + y = 1.
        if x + y > RANGE as i16 {
            let scale = RANGE as i16 / (x + y);
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
        (x + y, x - y)
    }

    /// Runs the `V` command to run the paddle at a specified relative velocity.
    ///
    /// # Parameters
    /// - `x`: The relative velocity of the paddle in the X direction in the range [-10^4, 10^4].
    /// - `y`: The relative velocity the paddle in the Y direction in the range [-10^4, 10^4].
    pub fn run_at_rel_velocity(&mut self, x: i16, y: i16) {
        let (a, b) = Self::corexy_to_cartesian(x, y);

        ufmt::uwriteln!(
            self.serial,
            "setting relative motor speeds a={}, b={}",
            x,
            y
        )
        .ok();
        self.stepper_a.run_at_speed(a);
        self.stepper_b.run_at_speed(b);
    }

    fn tick(&mut self) {
        self.stepper_a.poll();
        self.stepper_b.poll();
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
                    hw.run_at_rel_velocity(x, y);
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

    let stepper_a = Stepper::from_pins(
        pins.d2.into_output().downgrade(),
        pins.d5.into_output().downgrade(),
    );
    let stepper_b = Stepper::from_pins(
        pins.d3.into_output().downgrade(),
        pins.d6.into_output().downgrade(),
    );

    // Enable interrupts globally
    unsafe {
        // SAFETY: Not inside a critical section and any non-atomic operations have been completed
        // at this point.
        avr_device::interrupt::enable();
    }
    clock::prepare(dp.TC1);

    ufmt::uwriteln!(serial, "Ready!").ok();
    let mut hw = Hardware {
        serial,
        stepper_a,
        stepper_b,
    };
    start_recv(&mut hw)
}
