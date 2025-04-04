use crate::RANGE;
use arduino_hal::hal::port::{mode, Dynamic, Pin, PinOps};
use core::cmp::Ordering;
use embedded_hal::digital::{OutputPin, PinState, StatefulOutputPin};
use ufmt::{uDebug, uWrite, Formatter};

type Output<P = Dynamic> = Pin<mode::Output, P>;

/// The current active action of the stepper motor.
#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// The motor is stopped.
    #[default]
    Stopped,
    /// The motor is moving to a new position at a certain speed.
    MoveTo(
        /// The target step count, n, the motor is moving to.
        i32,
    ),
    /// The motor is moving endlessly in a certain direction at a certain speed.
    Direction(
        /// Whether the direction pin should be on HIGH.
        PinState,
    ),
}

impl uDebug for Action {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        match self {
            Self::Stopped => f.write_str("Stopped"),
            Self::MoveTo(target) => {
                f.write_str("MoveTo(")?;
                target.fmt(f)?;
                f.write_str(")")
            }
            Self::Direction(dir) => {
                f.write_str("Direction(")?;
                match dir {
                    PinState::Low => f.write_str("Low")?,
                    PinState::High => f.write_str("High")?,
                }
                f.write_str(")")
            }
        }
    }
}

/// Driver for the stepper motor.
pub struct Stepper<Step, Dir> {
    step: Output<Step>,
    dir: Output<Dir>,
    /// The current action the motor is performing.
    pub action: Action,
    /// The step count, n, the motor is currently on.
    pub steps: i32,
    /// The delay between each step, in microseconds.
    pub(crate) delay: u32,
    /// The time the motor last stepped, given in microseconds since program start.
    last_step: u32,

    _step_high: bool,
    _dir: PinState,
}

impl<Step: PinOps, Dir: PinOps> Stepper<Step, Dir> {
    /// Minimum delay between each step. (The "fastest" speed)
    ///
    /// The curve of "relative speed" follows a reciprocal function, where the delay is inversely
    /// proportional to the speed. ∆t = |MIN_DELAY_MICROS / speed|.
    pub const MIN_DELAY_MICROS: u32 = 30;

    /// Creates a new stepper motor instance from the given [`Pins`].
    pub fn from_pins(step: Output<Step>, dir: Output<Dir>) -> Self {
        let _step_high = step.is_set_high();
        let _dir = dir
            .is_set_high()
            .then_some(PinState::High)
            .unwrap_or(PinState::Low);
        Self {
            step,
            dir,
            action: Action::Stopped,
            steps: 0,
            delay: Self::MIN_DELAY_MICROS,
            last_step: 0,
            _step_high,
            _dir,
        }
    }

    /// Sets the "relative" speed of the motor. Must be in the range [0, 10^4].
    ///
    /// # Note
    /// - This does not move the motor, but rather sets the speed at which it will move.
    pub fn set_speed(&mut self, speed: u16) {
        debug_assert!(speed <= RANGE);

        if speed == 0 {
            self.stop();
        } else {
            self.delay = Self::MIN_DELAY_MICROS * RANGE as u32 / speed as u32;
        }
    }

    /// Move the motor to the given step count at the given speed.
    /// The speed should be in the range [0, 10^4], and the target should be in the range
    /// [-MAX_STEPS, MAX_STEPS].
    #[inline]
    pub fn move_to(&mut self, target: i32, speed: u16) {
        self.action = match speed {
            0 => Action::Stopped,
            speed => {
                self.set_speed(speed);
                Action::MoveTo(target)
            }
        };
    }

    /// The motor will simply begin moving at the given speed, but will not stop (unless it reaches
    /// a boundary).
    /// The speed should be in the range [-10^4, 10^4].
    #[inline]
    pub fn run_at_speed(&mut self, speed: i16) {
        let direction = speed
            .is_positive()
            .then_some(PinState::Low)
            .unwrap_or(PinState::High);
        self.action = Action::Direction(direction);
        self.set_speed(speed.unsigned_abs());
    }

    /// Stops the motor.
    #[inline]
    pub fn stop(&mut self) {
        self.action = Action::Stopped;
    }

    /// Whether the motor is currently stopped.
    #[inline]
    pub fn is_stopped(&self) -> bool {
        matches!(self.action, Action::Stopped)
    }

    /// Steps the motor by one step. This shouldn't be called directly, but rather through the
    /// `poll` method.
    ///
    /// # Note
    /// - This only updates [`steps`], not [`last_step`].
    /// - This does not check if the motor is at its bounds.
    /// - This does not check if the motor should be stopped nor consider the acceleration profile.
    #[inline]
    fn step(&mut self) {
        if let Action::MoveTo(n) = self.action
            && self.steps == n
        {
            self.action = Action::Stopped;
            return;
        }
        self._step_high = !self._step_high;
        if self._step_high {
            self.step.set_high();
        } else {
            self.step.set_low();
        }
        self.steps += self.direction();
    }

    /// Polls the motor and potentially steps it.
    ///
    /// # Returns
    /// The difference in steps since before the poll. This is typically -1, 0, or 1, given this
    /// function is called frequently enough. If the motor is stopped, this will always return 0.
    pub fn poll(&mut self, micros: u32) -> i32 {
        let direction = match self.action {
            Action::Stopped => return 0,
            Action::MoveTo(target) => match target.cmp(&self.steps) {
                Ordering::Less => PinState::Low,
                Ordering::Greater => PinState::High,
                Ordering::Equal => {
                    self.stop();
                    return 0;
                }
            },
            Action::Direction(dir) => dir,
        };
        match (self._dir, direction) {
            (PinState::High, PinState::Low) => self.dir.set_low(),
            (PinState::Low, PinState::High) => self.dir.set_high(),
            _ => {}
        }
        self._dir = direction;

        let steps_needed = match self.last_step {
            0 => 1,
            last_step => {
                let elapsed = micros.wrapping_sub(last_step);
                elapsed / self.delay
            }
        };
        for _ in 0..steps_needed.min(10) {
            self.step();
        }
        self.last_step = micros;
        steps_needed as i32 * self.direction()
    }

    /// Returns -1 if the motor is turning counter-clockwise, otherwise 1.
    #[inline]
    pub fn direction(&self) -> i32 {
        match self._dir {
            PinState::Low => -1,
            PinState::High => 1,
        }
    }
}
