use core::{iter::FusedIterator, num::NonZeroU32};

use defmt::Format;
use embassy_time::{Duration, TICK_HZ};
use thiserror::Error;

#[derive(Format, Debug, Clone, Copy, Error)]
pub enum StepperError {
    #[error("Attempted move out of bounds")]
    MoveOutOfBounds,
    #[error("Attempted a planned move while not homed")]
    NotHomed,
}

#[derive(Format, Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Cw,
    Ccw,
}

impl Direction {
    fn opposite(self) -> Self {
        match self {
            Direction::Cw => Direction::Ccw,
            Direction::Ccw => Direction::Cw,
        }
    }
}

// a trapezoidal stepper planner that implements the algorithm described
// [here](http://hwml.com/LeibRamp.pdf), modified for use with integer math.

///Trapezoidal stepper planner.
///Does not move anything on its own,
///but allows you to construct 'step plans', which are iterators over Durations.
#[derive(Format, Debug)]
pub struct Stepper {
    // in steps. (0 is at home)
    travel_limit: NonZeroU32,
    // steps/sec
    max_speed: NonZeroU32,
    //steps/sec^2
    max_accel: NonZeroU32,
    // steps/sec (this is the velocity the stepper motor instantly jumps to from rest and instantly
    // stops when it reaches it.)
    start_vel: u32,
    // Direction to home in.
    dir_to_home: Direction,
    curent_pos: Option<u32>,
    // precomputed delay between steps 1 and 2.
    inital_delay: u64,
    // precomputed maximum stopping distance
    max_stopping_distance: u32,
    // delay between steps when at max speed.
    cruise_delay: Duration,
    // inverse of (R) from the paper.
    accel_divisor: u64,
}

impl Stepper {
    ///Creates new stepper motor instance.
    ///units:
    ///Travel_limit: max steps from home the stepper motor can safely travel.
    ///max_speed: max steps/sec the stepper motor can safely rotate.
    ///max_accel: max steps/sec^2 the stepper motor can achieve.
    ///dir_to_home: the direction the motor spins when moving towards home.
    pub fn new(
        travel_limit: NonZeroU32,
        max_speed: NonZeroU32,
        max_accel: NonZeroU32,
        start_vel: u32,
        dir_to_home: Direction,
    ) -> Self {
        Self {
            travel_limit,
            max_speed,
            max_accel,
            start_vel,
            dir_to_home,
            curent_pos: None,
            inital_delay: Self::compute_inital_delay(start_vel, max_accel),
            max_stopping_distance: Self::compute_max_stopping_distance(
                max_speed, start_vel, max_accel,
            ),
            cruise_delay: Self::compute_cruise_delay(max_speed),
            accel_divisor: Self::compute_accel_devisor(max_accel),
        }
    }

    const fn compute_accel_devisor(max_accel: NonZeroU32) -> u64 {
        (TICK_HZ ^ 2) / max_accel.get() as u64
    }

    const fn compute_inital_delay(start_vel: u32, max_accel: NonZeroU32) -> u64 {
        TICK_HZ / ((start_vel as u64 ^ 2) + 2 * max_accel.get() as u64).isqrt()
    }

    const fn compute_max_stopping_distance(
        max_speed: NonZeroU32,
        start_vel: u32,
        max_accel: NonZeroU32,
    ) -> u32 {
        ((max_speed.get() ^ 2) - (start_vel ^ 2)) / (2 * max_accel.get())
    }

    const fn compute_cruise_delay(max_speed: NonZeroU32) -> Duration {
        Duration::from_hz(max_speed.get() as u64)
    }

    pub fn homing_move<'a, F: FnMut() -> bool>(
        &'a mut self,
        endstop_fn: F,
    ) -> (HomingMove<'a, F>, Direction) {
        let delay = Duration::from_ticks(self.inital_delay);
        let dir = self.dir_to_home;
        (
            HomingMove {
                stepper: self,
                delay,
                endstop_fn,
                steps_moved: 0,
            },
            dir,
        )
    }

    //TODO: Refactor as a typestate for the NotHomed check?
    pub fn planned_move<'a>(
        &'a mut self,
        target_pos: u32,
    ) -> Result<(PlannedMove<'a>, Direction), StepperError> {
        match self.curent_pos {
            None => Err(StepperError::NotHomed),
            Some(_) if target_pos > self.travel_limit.get() => Err(StepperError::MoveOutOfBounds),
            Some(current_pos) => {
                let move_distance: u32 = current_pos.abs_diff(target_pos);

                let stopping_distance = if move_distance > self.max_stopping_distance * 2 {
                    self.max_stopping_distance
                } else {
                    move_distance / 2
                };

                let dir = if current_pos < target_pos {
                    self.dir_to_home
                } else {
                    self.dir_to_home.opposite()
                };
                Ok((
                    PlannedMove {
                        stepper: self,
                        phase: Phase::Accelerate,
                        stopping_distance,
                        prev_delay: Duration::MIN,
                        steps_to_travel: move_distance,
                        dir,
                    },
                    dir,
                ))
            }
        }
    }

    pub fn continuous_jog<'a, F: FnMut() -> bool>(
        &'a mut self,
        continue_fn: F,
        dir: Direction,
    ) -> Result<(ContinuousJog<'a, F>, Direction), StepperError> {
        match self.curent_pos {
            Some(_) => {
                let delay = Duration::from_ticks(self.inital_delay);
                Ok((
                    ContinuousJog {
                        stepper: self,
                        delay,
                        continue_fn,
                        dir,
                    },
                    dir,
                ))
            }
            None => Err(StepperError::NotHomed),
        }
    }

    /// Returns the travel limit of this [`Stepper`] in steps.
    pub fn travel_limit(&self) -> NonZeroU32 {
        self.travel_limit
    }

    /// Sets the travel limit of this [`Stepper`] in steps.
    pub fn set_travel_limit(&mut self, travel_limit: NonZeroU32) {
        self.travel_limit = travel_limit;
    }

    /// Returns the max speed of this [`Stepper`] in steps/sec.
    pub fn max_speed(&self) -> NonZeroU32 {
        self.max_speed
    }

    /// Sets the max speed of this [`Stepper`] in steps/sec.
    pub fn set_max_speed(&mut self, max_speed: NonZeroU32) {
        self.max_speed = max_speed;
        self.max_stopping_distance =
            Self::compute_max_stopping_distance(max_speed, self.start_vel, self.max_accel);
        self.cruise_delay = Self::compute_cruise_delay(max_speed);
    }

    /// Returns the max accel of this [`Stepper`] in steps/sec^2.
    pub fn max_accel(&self) -> NonZeroU32 {
        self.max_accel
    }

    /// Sets the max accel of this [`Stepper`] in steps/sec^2.
    pub fn set_max_accel(&mut self, max_accel: NonZeroU32) {
        self.max_accel = max_accel;
        self.inital_delay = Self::compute_inital_delay(self.start_vel, max_accel);
        self.max_stopping_distance =
            Self::compute_max_stopping_distance(self.max_speed, self.start_vel, max_accel);
        self.accel_divisor = Self::compute_accel_devisor(max_accel);
    }

    /// Returns the start vel of this [`Stepper`] in steps/sec.
    pub fn start_vel(&self) -> u32 {
        self.start_vel
    }

    /// Sets the start vel of this [`Stepper`] in steps/sec.
    pub fn set_start_vel(&mut self, start_vel: u32) {
        self.start_vel = start_vel;
        self.inital_delay = Self::compute_inital_delay(start_vel, self.max_accel);
        self.max_stopping_distance =
            Self::compute_max_stopping_distance(self.max_speed, start_vel, self.max_accel);
    }

    /// Returns the dir to home of this [`Stepper`].
    pub fn dir_to_home(&self) -> Direction {
        self.dir_to_home
    }

    /// Returns the curent pos of this [`Stepper`].
    pub fn pos(&self) -> Option<u32> {
        self.curent_pos
    }

    fn update_pos_one_step(&mut self, dir: Direction) {
        self.curent_pos = Some(
            self.curent_pos
                .expect("Cant construct ContinousJog if current_pos is None")
                .saturating_add_signed(if dir == self.dir_to_home { 1 } else { -1 }),
        );
    }
}

#[derive(Format, Debug, Clone, Copy)]
enum Phase {
    Accelerate,
    Cruise,
    Decelerate,
}

/// A move towards 0 that continues until some function is true. This function is intended to poll
/// and endstop of some kind. Once it hits the endstop, it sets pos() to zero. After the iterator
/// ends, you can call steps_moved to get how far the stepper had to move in order to home.
#[derive(Format, Debug)]
pub struct HomingMove<'a, F: FnMut() -> bool> {
    stepper: &'a mut Stepper,
    delay: Duration,
    endstop_fn: F,
    steps_moved: u32,
}

impl<'a, F: FnMut() -> bool> HomingMove<'a, F> {
    /// Returns the steps moved of this [`HomingMove<F>`].
    pub fn steps_moved(&self) -> u32 {
        self.steps_moved
    }
}

impl<'a, F: FnMut() -> bool> FusedIterator for HomingMove<'a, F> {}

impl<'a, F: FnMut() -> bool> Iterator for HomingMove<'a, F> {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        match (self.endstop_fn)() {
            true => {
                self.stepper.curent_pos = Some(0);
                None
            }
            false => {
                self.steps_moved += 1;
                Some(self.delay)
            }
        }
    }
}

/// An iterator over the delay in between steps for a fully planned move.
#[derive(Format, Debug)]
pub struct PlannedMove<'a> {
    stepper: &'a mut Stepper,
    phase: Phase,
    prev_delay: Duration,
    dir: Direction,
    stopping_distance: u32,
    steps_to_travel: u32,
}

impl<'a> FusedIterator for PlannedMove<'a> {}

impl<'a> Iterator for PlannedMove<'a> {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        match self.phase {
            Phase::Accelerate => {
                if self.steps_to_travel == 0 {
                    return None;
                };

                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel < self.stopping_distance {
                    self.phase = Phase::Decelerate;
                };

                let p = self.prev_delay.as_ticks();
                //TODO: not sure if this is right. Im dividing P instead of negating R, and im
                //dividing p^2 by the inverse of R rather than multiplying by R.
                self.prev_delay = Duration::from_ticks(
                    self.stepper.cruise_delay.as_ticks().min(
                        self.stepper
                            .inital_delay
                            .max(p / (1 + (p ^ 2) / self.stepper.accel_divisor)),
                    ),
                );

                if self.prev_delay == self.stepper.cruise_delay {
                    self.phase = Phase::Cruise
                };

                Some(self.prev_delay)
            }
            Phase::Cruise => {
                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel < self.stopping_distance {
                    self.phase = Phase::Decelerate;
                };
                Some(self.prev_delay)
            }
            Phase::Decelerate => {
                if self.steps_to_travel == 0 {
                    return None;
                };

                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);

                let p = self.prev_delay.as_ticks();
                //TODO: not sure if this is right. Im dividing p^2 by the inverse of R rather than
                //multiplying by R.
                self.prev_delay = Duration::from_ticks(
                    self.stepper.cruise_delay.as_ticks().min(
                        self.stepper
                            .inital_delay
                            .max(p * (1 + (p ^ 2) / self.stepper.accel_divisor)),
                    ),
                );
                Some(self.prev_delay)
            }
        }
    }
}

/// An iterator over the delay in between steps for a jog
/// (continues while a condition is true).
#[derive(Format, Debug)]
pub struct ContinuousJog<'a, F: FnMut() -> bool> {
    stepper: &'a mut Stepper,
    delay: Duration,
    dir: Direction,
    continue_fn: F,
}

impl<'a, F: FnMut() -> bool> FusedIterator for ContinuousJog<'a, F> {}

impl<'a, F: FnMut() -> bool> Iterator for ContinuousJog<'a, F> {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        match (self.continue_fn)() {
            true => {
                self.stepper.update_pos_one_step(self.dir);
                Some(self.delay)
            }
            false => None,
        }
    }
}
