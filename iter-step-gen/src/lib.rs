#![cfg_attr(not(test), no_std)]

use core::{
    cmp::{max, min},
    iter::FusedIterator,
    num::NonZeroU32,
};

use defmt::Format;
use embassy_time::{Duration, TICK_HZ};
use thiserror::Error;

#[derive(Format, Debug, Clone, Copy, Error, PartialEq, Eq)]
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
// [here](http://hwml.com/LeibRamp.pdf), heavily modified for use with integer math.
// the modifications are explained in the math.typ file in this package.

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
    // precomputed maximum stopping distance
    max_stopping_distance: u32,
    // delay between steps when at max speed.
    cruise_delay: Duration,
    // precomputed divisor for acceleration calcs.
    accel_divisor: u64,
    // precomputed delay of the first step
    inital_delay: u64,
}

impl Stepper {
    ///Creates new stepper motor instance.
    ///units:
    ///Travel_limit: max steps from home the stepper motor can safely travel.
    ///max_speed: max steps/sec the stepper motor can safely rotate.
    ///max_accel: max steps/sec^2 the stepper motor can achieve.
    ///dir_to_home: the direction the motor spins when moving towards home.
    pub const fn new(
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
            max_stopping_distance: Self::compute_max_stopping_distance(
                max_speed, start_vel, max_accel,
            ),
            cruise_delay: Self::compute_cruise_delay(max_speed),
            accel_divisor: Self::compute_accel_divisor(max_accel),
            inital_delay: Self::compute_inital_delay(start_vel, max_accel),
        }
    }

    const fn compute_accel_divisor(max_accel: NonZeroU32) -> u64 {
        TICK_HZ.pow(2) / max_accel.get() as u64
    }

    const fn compute_inital_delay(start_vel: u32, max_accel: NonZeroU32) -> u64 {
        TICK_HZ / ((start_vel as u64).pow(2) + 2 * max_accel.get() as u64).isqrt()
    }

    const fn compute_max_stopping_distance(
        max_speed: NonZeroU32,
        start_vel: u32,
        max_accel: NonZeroU32,
    ) -> u32 {
        (max_speed
            .get()
            .saturating_pow(2)
            .saturating_sub(start_vel.saturating_pow(2)))
            / (2 * max_accel.get())
    }

    const fn compute_cruise_delay(max_speed: NonZeroU32) -> Duration {
        Duration::from_hz(max_speed.get() as u64)
    }

    pub fn homing_move<'a, F: FnMut() -> bool>(
        &'a mut self,
        endstop_fn: F,
    ) -> (HomingMove<'a, F>, Direction) {
        let delay = Duration::from_ticks(TICK_HZ / (self.start_vel as u64));
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

                // TODO: Not sure why I need that +2, but somewhere we have an off-by-2, as without
                // this we have too much deccel on the last step of a move.
                let stopping_distance = if move_distance > self.max_stopping_distance * 2 {
                    self.max_stopping_distance
                } else {
                    move_distance.div_ceil(2)
                } + 2;

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
                        prev_delay: Duration::MAX,
                        steps_to_travel: move_distance,
                        dir,
                        rem: 0,
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
                let delay = Duration::from_ticks(TICK_HZ / (self.start_vel as u64));
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
        self.max_stopping_distance =
            Self::compute_max_stopping_distance(self.max_speed, self.start_vel, max_accel);
        self.accel_divisor = Self::compute_accel_divisor(max_accel);
        self.inital_delay = Self::compute_inital_delay(self.start_vel, max_accel);
    }

    /// Returns the start vel of this [`Stepper`] in steps/sec.
    pub fn start_vel(&self) -> u32 {
        self.start_vel
    }

    /// Sets the start vel of this [`Stepper`] in steps/sec.
    pub fn set_start_vel(&mut self, start_vel: u32) {
        self.start_vel = start_vel;
        self.max_stopping_distance =
            Self::compute_max_stopping_distance(self.max_speed, start_vel, self.max_accel);
        self.inital_delay = Self::compute_inital_delay(start_vel, self.max_accel);
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
    rem: u64,
}

impl<'a> FusedIterator for PlannedMove<'a> {}

impl<'a> Iterator for PlannedMove<'a> {
    type Item = Duration;

    // TODO: For some reason the acceleration curve is asymetrical, and goes over the set
    // acceleration sometimes? the output is 'jagged'...
    fn next(&mut self) -> Option<Self::Item> {
        match self.phase {
            Phase::Accelerate => {
                if self.steps_to_travel == 0 {
                    return None;
                };

                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel <= self.stopping_distance {
                    self.phase = Phase::Decelerate;
                    self.rem = 0;
                };

                let p = self.prev_delay.as_ticks();
                let pdividend = p.saturating_pow(3) + self.rem;
                let pdiff = pdividend / self.stepper.accel_divisor;
                self.rem = pdividend % self.stepper.accel_divisor;
                self.prev_delay = Duration::from_ticks(min(
                    max(
                        p.saturating_sub(pdiff),
                        self.stepper.cruise_delay.as_ticks(),
                    ),
                    self.stepper.inital_delay,
                ));

                if self.prev_delay == self.stepper.cruise_delay {
                    self.phase = Phase::Cruise
                };

                Some(self.prev_delay)
            }
            Phase::Cruise => {
                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel <= self.stopping_distance {
                    self.phase = Phase::Decelerate;
                    self.rem = 0;
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
                let pdividend = p.saturating_pow(3) + self.rem;
                let pdiff = pdividend / self.stepper.accel_divisor;
                self.rem = pdividend % self.stepper.accel_divisor;
                self.prev_delay = Duration::from_ticks(min(
                    max(
                        p.saturating_add(pdiff),
                        self.stepper.cruise_delay.as_ticks(),
                    ),
                    self.stepper.inital_delay,
                ));
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

#[cfg(test)]
mod test {
    use core::num::NonZeroU32;

    use embassy_time::{Duration, TICK_HZ};

    use crate::{Direction, Stepper, StepperError};

    const TRAVEL_LIMIT: NonZeroU32 = NonZeroU32::new(2048).unwrap();
    const MAX_VEL: NonZeroU32 = NonZeroU32::new(255).unwrap();
    const MAX_ACCEL: NonZeroU32 = NonZeroU32::new(64).unwrap();
    const START_VEL: u32 = 50;
    const DIR: Direction = Direction::Cw;

    #[test]
    fn test_home() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL, DIR);
        assert_eq!(stepper.curent_pos, None);

        let mut endstop = [false, false, true].into_iter();
        let (steps, direction) = stepper.homing_move(|| endstop.next().unwrap());

        assert_eq!(direction, DIR);
        for step in steps {
            assert_eq!(step, Duration::from_hz(START_VEL as u64));
            println!("{}", (TICK_HZ / step.as_ticks()));
        }
        assert_eq!(stepper.curent_pos, Some(0));
    }

    #[test]
    fn test_move_travel_guards() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL, DIR);
        assert_eq!(
            stepper.planned_move(100).unwrap_err(),
            StepperError::NotHomed
        );
        let (mut steps, _) = stepper.homing_move(|| true);
        steps.next();
        assert_eq!(
            stepper.planned_move(TRAVEL_LIMIT.get() + 1).unwrap_err(),
            StepperError::MoveOutOfBounds
        );
    }

    #[test]
    fn test_move_max_vel() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL, DIR);
        let (mut steps, _) = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let (steps, _) = stepper.planned_move(TRAVEL_LIMIT.get()).unwrap();
        print!("speed,delay");
        for step in steps {
            println!("{},{}", (TICK_HZ / step.as_ticks()), step.as_ticks());
            assert!(step >= Duration::from_hz(MAX_VEL.get().into()));
        }
        assert_eq!(stepper.curent_pos, Some(TRAVEL_LIMIT.get()));
    }

    #[test]
    fn test_move_max_accel() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL, DIR);
        let (mut steps, _) = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let mut prev_step = stepper.inital_delay;
        let mut time = Duration::from_ticks(0);

        let mut accels: [f64; _] = [0.0; 2];
        let mut accel_indx = 0;

        let (steps, _) = stepper.planned_move(TRAVEL_LIMIT.get()).unwrap();
        println!("time,delay,vel,accel,avg_accel");
        for step in steps {
            let prev_vel = TICK_HZ as f64 / prev_step as f64;
            let vel = TICK_HZ as f64 / step.as_ticks() as f64;
            let accel = (vel - prev_vel) * prev_vel;
            accels[accel_indx] = accel;
            accel_indx = (accel_indx + 1) % accels.len();
            let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
            println!(
                "{},{},{},{},{}",
                time.as_ticks(),
                step.as_ticks(),
                vel,
                accel,
                avg,
            );

            // due to the fact we are using a first degree approximation of the ideal formula
            // (which requires a square root), we sometimes go up 1% over our max acceleration.
            // Also, for some reason there are single-step spikes, but they dissapear when taking a
            // 2 step moving average.
            assert!(avg.abs() <= MAX_ACCEL.get() as f64 + (MAX_ACCEL.get() as f64 / 1.0));

            time += step;
            prev_step = step.as_ticks();
        }

        let final_vel = TICK_HZ as f64 / prev_step as f64;
        let final_accel = (stepper.start_vel as f64 - final_vel) * final_vel;
        accels[accel_indx] = final_accel;
        let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
        println!(
            "{},{},{},{},{}",
            time.as_ticks(),
            prev_step,
            stepper.start_vel,
            final_accel,
            avg,
        );

        assert!(final_accel.abs() <= MAX_ACCEL.get() as f64 + 1.0);
        assert_eq!(stepper.curent_pos, Some(TRAVEL_LIMIT.get()));
    }

    #[test]
    fn test_move_max_accel_short() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL, DIR);
        let (mut steps, _) = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let mut prev_step = stepper.inital_delay;
        let mut time = Duration::from_ticks(0);

        let mut accels: [f64; _] = [0.0; 2];
        let mut accel_indx = 0;

        let (steps, _) = stepper.planned_move(MAX_ACCEL.get()).unwrap();
        println!("time,delay,vel,accel,avg_accel");
        for step in steps {
            let prev_vel = TICK_HZ as f64 / prev_step as f64;
            let vel = TICK_HZ as f64 / step.as_ticks() as f64;
            let accel = (vel - prev_vel) * prev_vel;
            accels[accel_indx] = accel;
            accel_indx = (accel_indx + 1) % accels.len();
            let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
            println!(
                "{},{},{},{},{}",
                time.as_ticks(),
                step.as_ticks(),
                vel,
                accel,
                avg,
            );

            // due to the fact we are using a first degree approximation of the ideal formula
            // (which requires a square root), we sometimes go up 1% over our max acceleration.
            // Also, for some reason there are single-step spikes, but they dissapear when taking a
            // 2 step moving average.
            assert!(avg.abs() <= MAX_ACCEL.get() as f64 + (MAX_ACCEL.get() as f64 / 1.0));

            time += step;
            prev_step = step.as_ticks();
        }

        let final_vel = TICK_HZ as f64 / prev_step as f64;
        let final_accel = (stepper.start_vel as f64 - final_vel) * final_vel;
        accels[accel_indx] = final_accel;
        let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
        println!(
            "{},{},{},{},{}",
            time.as_ticks(),
            prev_step,
            stepper.start_vel,
            final_accel,
            avg,
        );

        assert!(final_accel.abs() <= MAX_ACCEL.get() as f64 + 1.0);
        assert_eq!(stepper.curent_pos, Some(MAX_ACCEL.get()));
    }
}
