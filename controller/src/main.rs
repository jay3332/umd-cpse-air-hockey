//! Receives all commands from the serial port (USART1) and sends them to the peripheral devices.

#![no_std]
#![no_main]

use arduino_hal::{Peripherals, Usart, hal::{Atmega, usart::{BaudrateArduinoExt, UsartOps}}};
use panic_halt as _;

/// Bitrate for the serial pin.
/// Current value taken from <https://github.com/jjrobots/Air_Hockey_Robot_EVO/blob/master/Arduino/AHRobot_EVO/AHRobot_EVO.ino#L38>
const BITRATE: u32 = 115_200;

/// Maximum motor speed, in steps/seg.
const MOTOR_MAX_SPEED: i32 = 32_000;

trait SerialExt {
    fn read_u32(&mut self) -> u32;
    fn read_i32(&mut self) -> i32;
}

impl<U: UsartOps<Atmega, Rx, Tx>, Rx, Tx> SerialExt for Usart<U, Rx, Tx> {
    fn read_u32(&mut self) -> u32 {
        u32::from_le_bytes([self.read_byte(); 4])
    }

    fn read_i32(&mut self) -> i32 {
        i32::from_le_bytes([self.read_byte(); 4])
    }
}

/// Sets the relative velocity of the paddle to <X, Y>, where X and Y are in the range [-1, 1].
/// The speed is relative to the maximum speed of the motor.
fn set_paddle_motor_speeds(mut x: f32, mut y: f32) {
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
    let a = (x + y) * MOTOR_MAX_SPEED as f32;
    let b = (x - y) * MOTOR_MAX_SPEED as f32;


}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().expect("no arduino device?");
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::Usart::new(
        dp.USART1,
        pins.d0,
        pins.d1.into_output(),
        BaudrateArduinoExt::into_baudrate(BITRATE),
    );

    // Serial commands
    // ===============
    // NOTE: All integers are in little-endian format.
    //
    // - V[X:i32][Y:i32] -> Set the velocity (relative speed) of the paddle to <X/10^6, Y/10^6>.
    //   Constraints: -10^6 <= X <= 10^6 and -10^6 <= Y <= 10^6.
    loop {
        match serial.read_byte() {
            b'V' => {
                const RANGE: i32 = 1_000_000;

                let x = serial.read_i32().clamp(-RANGE, RANGE);
                let y = serial.read_i32().clamp(-RANGE, RANGE);
                set_paddle_motor_speeds(x as f32 / RANGE as f32, y as f32 / RANGE as f32);
            }
        }
    }
}
