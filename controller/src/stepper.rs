use arduino_hal::{
    hal::port::{mode, Dynamic, Pin},
    Pins,
};

/// Steps per unit/second for the stepper motor.
const STEPS_PER_UNIT: u32 = 1000;

/// Minimum delay between each step, in microseconds. (The "fastest" speed)
const MIN_DELAY_US: u32 = 45;

type Output<P = Dynamic> = Pin<mode::Output, P>;

/// Stepper motor wrapper.
pub struct Stepper {
    step: Output,
    dir: Output,
    /// The delay between each step, in microseconds.
    pub delay_us: u32,
    /// The total number of steps taken.
    pub steps: u32,
}

impl Stepper {
    /// Creates a new stepper motor instance from the given [`Pins`].
    pub fn from_pins(step: Output, dir: Output) -> Self {
        Self {
            step,
            dir,
            delay_us: 0,
            steps: 0,
        }
    }

    pub fn set_speed(&mut self, mut speed: f32) {
        if speed == 0.0 {
            self.delay_us = 0;
            return;
        }

        if speed.is_sign_negative() {
            self.dir.set_low();
            speed = -speed;
        } else {
            self.dir.set_high();
        }

        self.delay_us = (45.0 / speed) as u32;
    }

    pub fn step(&mut self) {
        if self.delay_us == 0 {
            return;
        }
        self.step.set_high();
        arduino_hal::delay_us(self.delay_us);
        self.step.set_low();
        arduino_hal::delay_us(self.delay_us);
        self.steps += 1;
    }
}
