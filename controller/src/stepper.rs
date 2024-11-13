use arduino_hal::hal::port::{mode, Dynamic, Pin};
use core::num::NonZeroU32;

type Output<P = Dynamic> = Pin<mode::Output, P>;

/// Stepper motor wrapper.
pub struct Stepper {
    step: Output,
    dir: Output,
    /// The delay between each step, in microseconds. ``None`` means the motor is stopped.
    pub delay_us: Option<NonZeroU32>,
    /// The step count, n, the motor is currently on.
    pub steps: i32,
}

impl Stepper {
    /// Steps per unit for the stepper motor.
    ///
    /// Note: Currently, the "unit" is arbitrary but later will be calibrated to ~1cm.
    pub const STEPS_PER_UNIT: u32 = 1000;

    /// Minimum delay between each step, in microseconds. (The "fastest" speed)
    ///
    /// The curve of "relative speed" follows a reciprocal function, where the delay is inversely
    /// proportional to the speed. ∆t = |MIN_DELAY_US / speed|.
    pub const MIN_DELAY_US: f32 = 45.0;

    /// The absolute rotary bounds of the stepper motors, in steps.
    /// The motor will not move beyond these bounds.
    pub const MAX_STEPS: i32 = 10_000;

    /// The absolute "relative" speed by which the motor will stop moving.
    pub const EPSILON: f32 = 0.001;

    /// Creates a new stepper motor instance from the given [`Pins`].
    pub fn from_pins(step: Output, dir: Output) -> Self {
        Self {
            step,
            dir,
            delay_us: None,
            steps: 0,
        }
    }

    /// Sets the "relative" speed of the motor. Must be in the range [-1, 1].
    pub fn set_speed(&mut self, mut speed: f32) {
        if speed < Self::EPSILON && speed > -Self::EPSILON {
            self.delay_us = None;
            return;
        }
        debug_assert!(speed >= -1.0 && speed <= 1.0);

        if speed.is_sign_negative() {
            self.dir.set_low();
            speed = -speed;
        } else {
            self.dir.set_high();
        }

        self.delay_us = NonZeroU32::new((Self::MIN_DELAY_US / speed) as u32);
    }

    // TODO this is blocking
    pub fn step(&mut self) {
        if self.steps <= -Self::MAX_STEPS || self.steps >= Self::MAX_STEPS {
            return;
        }
        if let Some(delay_us) = self.delay_us {
            self.step.toggle();
            arduino_hal::delay_us(delay_us.get());
            self.steps += self.direction();
        }
    }

    /// Returns -1 if the motor is turning counter-clockwise, otherwise 1.
    #[inline]
    pub fn direction(&self) -> i32 {
        if self.dir.is_set_high() {
            1
        } else {
            -1
        }
    }
}
