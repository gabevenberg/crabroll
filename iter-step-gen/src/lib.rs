#![cfg_attr(not(test), no_std)]

use core::{
    cmp::{max, min},
    iter::FusedIterator,
    num::NonZeroU32,
};

//TODO: Move defmt stuff into crate feature.
use defmt::Format;
//TODO: use core::Duration instead of embassy_time duration to remove dep on embassy.
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
    ToHome,
    AwayFromHome,
}

// a trapezoidal stepper planner that implements the algorithm described [here](http://hwml.com/LeibRamp.pdf),
// heavily modified for use with integer math.
// the modifications are explained in the IntLeibRamp.typ file in this package.

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
    // in steps from home. (None until homed)
    current_pos: Option<u32>,
    // precomputed length of the deceleration ramp, in steps.
    max_stopping_distance: u32,
    // precomputed length of the acceleration ramp, in steps.
    max_accel_distance: u32,
    // delay between steps when at max speed.
    cruise_delay: Duration,
    // precomputed divisor for acceleration calc.
    accel_divisor: u64,
    // precomputed delay of the first step
    initial_delay: u64,
}

impl Stepper {
    ///Creates new stepper motor instance.
    ///units:
    ///* `travel_limit`: max steps from home the stepper motor can safely travel.
    ///* `max_speed`: max steps/sec the stepper motor can safely rotate.
    ///* `max_accel`: max steps/sec^2 the stepper motor can achieve.
    ///* `start_vel`: steps/sec the stepper motor can jump to from rest, and stop from instantly.
    #[must_use]
    pub const fn new(
        travel_limit: NonZeroU32,
        max_speed: NonZeroU32,
        max_accel: NonZeroU32,
        start_vel: u32,
    ) -> Self {
        Self {
            travel_limit,
            max_speed,
            max_accel,
            start_vel,
            current_pos: None,
            max_stopping_distance: Self::compute_max_stopping_distance(
                travel_limit,
                max_speed,
                start_vel,
                max_accel,
            ),
            max_accel_distance: Self::compute_max_accel_distance(
                travel_limit,
                max_speed,
                start_vel,
                max_accel,
            ),
            cruise_delay: Self::compute_cruise_delay(max_speed),
            accel_divisor: Self::compute_accel_divisor(max_accel),
            initial_delay: Self::compute_initial_delay(start_vel, max_accel),
        }
    }

    const fn compute_accel_divisor(max_accel: NonZeroU32) -> u64 {
        TICK_HZ.pow(2) / max_accel.get() as u64
    }

    const fn compute_initial_delay(start_vel: u32, max_accel: NonZeroU32) -> u64 {
        // p1 = F/sqrt(v0^2 + 2a), evaluated as sqrt(F^2/(v0^2 + 2a)) so the speed is not rounded
        // down to a whole step/sec before the division.
        (TICK_HZ.pow(2) / ((start_vel as u64).pow(2) + 2 * max_accel.get() as u64)).isqrt()
    }

    /// Length of the deceleration ramp, in steps.
    /// The ideal formula for this is `(max_speed^2 - start_vel^2)/(2*max_accel)`,
    /// but the integer ramp does not follow the ideal curve exactly,
    /// so we walk the ramp instead of trusting the formula.
    const fn compute_max_stopping_distance(
        travel_limit: NonZeroU32,
        max_speed: NonZeroU32,
        start_vel: u32,
        max_accel: NonZeroU32,
    ) -> u32 {
        let accel_divisor = Self::compute_accel_divisor(max_accel);
        let initial_delay = Self::compute_initial_delay(start_vel, max_accel);
        let mut delay = Self::compute_cruise_delay(max_speed).as_ticks();
        let mut rem = 0;
        let mut steps = 0;
        // a ramp longer than the whole axis can never be run,
        // so there is no point counting past it (and it keeps this loop bounded for a misconfigured stepper).
        while delay < initial_delay && steps < travel_limit.get() {
            let (delay_diff, new_rem) = ramp_step(delay, rem, accel_divisor);
            rem = new_rem;
            delay = delay.saturating_add(delay_diff);
            steps += 1;
        }
        steps
    }

    /// Length of the acceleration ramp, in steps,
    /// counted the same way as `compute_max_stopping_distance` so the two are comparable.
    /// The first step of a move jumps straight to `initial_delay` instead of ramping,
    /// so a move needs one more step than this to reach `max_speed`.
    const fn compute_max_accel_distance(
        travel_limit: NonZeroU32,
        max_speed: NonZeroU32,
        start_vel: u32,
        max_accel: NonZeroU32,
    ) -> u32 {
        let accel_divisor = Self::compute_accel_divisor(max_accel);
        let cruise_delay = Self::compute_cruise_delay(max_speed).as_ticks();
        let mut delay = Self::compute_initial_delay(start_vel, max_accel);
        let mut rem = 0;
        let mut steps = 0;
        while delay > cruise_delay && steps < travel_limit.get() {
            let (delay_diff, new_rem) = ramp_step(delay, rem, accel_divisor);
            rem = new_rem;
            delay = delay.saturating_sub(delay_diff);
            steps += 1;
        }
        steps
    }

    /// How many of a `move_distance` step move to spend decelerating.
    fn stopping_distance(&self, move_distance: u32) -> u32 {
        // The two ramps are not the same length:
        // the approximation overshoots while accelerating and undershoots while decelerating,
        // so deceleration needs a few more steps than acceleration to cover the same speed range.
        // Give deceleration those steps out of the acceleration half of the move,
        // or a move too short to reach max_speed runs out of steps before the ramp is done and the last step has to stop from well above start_vel.
        let ramp_lag = self
            .max_stopping_distance
            .saturating_sub(self.max_accel_distance);
        min(
            self.max_stopping_distance,
            move_distance.saturating_add(ramp_lag).div_ceil(2),
        )
    }

    const fn compute_cruise_delay(max_speed: NonZeroU32) -> Duration {
        Duration::from_hz(max_speed.get() as u64)
    }

    pub fn homing_move<F: FnMut() -> bool>(&mut self, endstop_fn: F) -> HomingMove<'_, F> {
        self.current_pos = None;
        let delay = Duration::from_ticks(TICK_HZ / u64::from(self.start_vel));
        HomingMove {
            stepper: self,
            delay,
            endstop_fn,
            steps_moved: 0,
        }
    }

    //TODO: Refactor as a typestate for the NotHomed check?
    pub fn planned_move(
        &mut self,
        target_pos: u32,
    ) -> Result<(PlannedMove<'_>, Direction), StepperError> {
        match self.current_pos {
            None => Err(StepperError::NotHomed),
            Some(_) if target_pos > self.travel_limit.get() => Err(StepperError::MoveOutOfBounds),
            Some(current_pos) => {
                let move_distance: u32 = current_pos.abs_diff(target_pos);

                let stopping_distance = self.stopping_distance(move_distance);

                let dir = if current_pos < target_pos {
                    Direction::AwayFromHome
                } else {
                    Direction::ToHome
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

    pub fn continuous_jog<F: FnMut() -> bool>(
        &mut self,
        continue_fn: F,
        dir: Direction,
    ) -> Result<ContinuousJog<'_, F>, StepperError> {
        match self.current_pos {
            Some(_) => {
                let delay = Duration::from_ticks(TICK_HZ / u64::from(self.start_vel));
                Ok(ContinuousJog {
                    stepper: self,
                    delay,
                    continue_fn,
                    dir,
                })
            }
            None => Err(StepperError::NotHomed),
        }
    }

    /// Returns the travel limit of this [`Stepper`] in steps.
    #[must_use]
    pub fn travel_limit(&self) -> NonZeroU32 {
        self.travel_limit
    }

    /// Sets the travel limit of this [`Stepper`] in steps.
    pub fn set_travel_limit(&mut self, travel_limit: NonZeroU32) {
        self.travel_limit = travel_limit;
        self.max_stopping_distance = Self::compute_max_stopping_distance(
            travel_limit,
            self.max_speed,
            self.start_vel,
            self.max_accel,
        );
        self.max_accel_distance = Self::compute_max_accel_distance(
            travel_limit,
            self.max_speed,
            self.start_vel,
            self.max_accel,
        );
    }

    /// Returns the max speed of this [`Stepper`] in steps/sec.
    #[must_use]
    pub fn max_speed(&self) -> NonZeroU32 {
        self.max_speed
    }

    /// Sets the max speed of this [`Stepper`] in steps/sec.
    pub fn set_max_speed(&mut self, max_speed: NonZeroU32) {
        self.max_speed = max_speed;
        self.max_stopping_distance = Self::compute_max_stopping_distance(
            self.travel_limit,
            max_speed,
            self.start_vel,
            self.max_accel,
        );
        self.max_accel_distance = Self::compute_max_accel_distance(
            self.travel_limit,
            max_speed,
            self.start_vel,
            self.max_accel,
        );
        self.cruise_delay = Self::compute_cruise_delay(max_speed);
    }

    /// Returns the max accel of this [`Stepper`] in steps/sec^2.
    #[must_use]
    pub fn max_accel(&self) -> NonZeroU32 {
        self.max_accel
    }

    /// Sets the max accel of this [`Stepper`] in steps/sec^2.
    pub fn set_max_accel(&mut self, max_accel: NonZeroU32) {
        self.max_accel = max_accel;
        self.max_stopping_distance = Self::compute_max_stopping_distance(
            self.travel_limit,
            self.max_speed,
            self.start_vel,
            max_accel,
        );
        self.max_accel_distance = Self::compute_max_accel_distance(
            self.travel_limit,
            self.max_speed,
            self.start_vel,
            max_accel,
        );
        self.accel_divisor = Self::compute_accel_divisor(max_accel);
        self.initial_delay = Self::compute_initial_delay(self.start_vel, max_accel);
    }

    /// Returns the start vel of this [`Stepper`] in steps/sec.
    #[must_use]
    pub fn start_vel(&self) -> u32 {
        self.start_vel
    }

    /// Sets the start vel of this [`Stepper`] in steps/sec.
    pub fn set_start_vel(&mut self, start_vel: u32) {
        self.start_vel = start_vel;
        self.max_stopping_distance = Self::compute_max_stopping_distance(
            self.travel_limit,
            self.max_speed,
            start_vel,
            self.max_accel,
        );
        self.max_accel_distance = Self::compute_max_accel_distance(
            self.travel_limit,
            self.max_speed,
            start_vel,
            self.max_accel,
        );
        self.initial_delay = Self::compute_initial_delay(start_vel, self.max_accel);
    }

    /// Returns the current pos of this [`Stepper`].
    #[must_use]
    pub fn pos(&self) -> Option<u32> {
        self.current_pos
    }

    fn update_pos_one_step(&mut self, dir: Direction) {
        self.current_pos = Some(
            self.current_pos
                .expect("Attempted to update position while not homed.")
                .saturating_add_signed(if dir == Direction::AwayFromHome {
                    1
                } else {
                    -1
                }),
        );
    }
}

#[derive(Format, Debug, Clone, Copy)]
enum Phase {
    Accelerate,
    Cruise,
    Decelerate,
}

/// A move towards 0 that continues until some function is true.
/// This function is intended to poll an endstop of some kind.
/// Once it hits the endstop, it sets `pos()` to zero.
/// After the iterator ends,
/// you can call `steps_moved` to get how far the stepper had to move in order to home.
#[derive(Format, Debug)]
pub struct HomingMove<'a, F: FnMut() -> bool> {
    stepper: &'a mut Stepper,
    delay: Duration,
    endstop_fn: F,
    steps_moved: u32,
}

impl<F: FnMut() -> bool> HomingMove<'_, F> {
    /// Returns the steps moved of this [`HomingMove<F>`].
    pub fn steps_moved(&self) -> u32 {
        self.steps_moved
    }
}

impl<F: FnMut() -> bool> FusedIterator for HomingMove<'_, F> {}

impl<F: FnMut() -> bool> Iterator for HomingMove<'_, F> {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        if self.stepper.current_pos.is_none() {
            if (self.endstop_fn)() {
                self.stepper.current_pos = Some(0);
                None
            } else {
                self.steps_moved += 1;
                Some(self.delay)
            }
        } else {
            None
        }
    }
}

/// One `LeibRamp` update.
/// Takes the current delay period in ticks and the remainder carried from the last update,
/// and returns the magnitude of the change in delay period along with the new remainder.
/// Subtract the change to accelerate and add it to decelerate.
const fn ramp_step(p: u64, rem: u64, accel_divisor: u64) -> (u64, u64) {
    let pdividend = p.saturating_pow(3).saturating_add(rem);
    (pdividend / accel_divisor, pdividend % accel_divisor)
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

impl PlannedMove<'_> {
    /// One `LeibRamp` update, carrying the remainder forward in `self`.
    fn ramp_step(&mut self, p: u64) -> u64 {
        let (delay_diff, rem) = ramp_step(p, self.rem, self.stepper.accel_divisor);
        self.rem = rem;
        delay_diff
    }
}

impl FusedIterator for PlannedMove<'_> {}

impl Iterator for PlannedMove<'_> {
    type Item = Duration;

    // TODO: For some reason the acceleration curve goes over the set acceleration sometimes?
    // the output is 'jagged'...
    fn next(&mut self) -> Option<Self::Item> {
        match self.phase {
            Phase::Accelerate => {
                if self.steps_to_travel == 0 {
                    return None;
                }

                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel <= self.stopping_distance {
                    self.phase = Phase::Decelerate;
                    self.rem = 0;
                }

                let p = self.prev_delay.as_ticks();
                let pdiff = self.ramp_step(p);
                self.prev_delay = Duration::from_ticks(min(
                    max(
                        p.saturating_sub(pdiff),
                        self.stepper.cruise_delay.as_ticks(),
                    ),
                    self.stepper.initial_delay,
                ));

                if self.prev_delay == self.stepper.cruise_delay {
                    self.phase = Phase::Cruise;
                }

                Some(self.prev_delay)
            }
            Phase::Cruise => {
                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);
                if self.steps_to_travel <= self.stopping_distance {
                    self.phase = Phase::Decelerate;
                    self.rem = 0;
                }
                Some(self.prev_delay)
            }
            Phase::Decelerate => {
                if self.steps_to_travel == 0 {
                    return None;
                }

                self.steps_to_travel -= 1;
                self.stepper.update_pos_one_step(self.dir);

                let p = self.prev_delay.as_ticks();
                let pdiff = self.ramp_step(p);
                self.prev_delay = Duration::from_ticks(min(
                    max(
                        p.saturating_add(pdiff),
                        self.stepper.cruise_delay.as_ticks(),
                    ),
                    self.stepper.initial_delay,
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

impl<F: FnMut() -> bool> Iterator for ContinuousJog<'_, F> {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        if (self.continue_fn)() {
            self.stepper.update_pos_one_step(self.dir);
            Some(self.delay)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    #![allow(clippy::cast_precision_loss)]
    use core::num::NonZeroU32;

    use embassy_time::{Duration, TICK_HZ};

    use crate::{Stepper, StepperError};

    const TRAVEL_LIMIT: NonZeroU32 = NonZeroU32::new(2048).unwrap();
    const MAX_VEL: NonZeroU32 = NonZeroU32::new(255).unwrap();
    const MAX_ACCEL: NonZeroU32 = NonZeroU32::new(64).unwrap();
    const START_VEL: u32 = 50;

    #[test]
    fn test_home() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
        assert_eq!(stepper.current_pos, None);

        let mut endstop = [false, false, true].into_iter();
        let steps = stepper.homing_move(|| endstop.next().unwrap());

        for step in steps {
            assert_eq!(step, Duration::from_hz(u64::from(START_VEL)));
            println!("{}", (TICK_HZ / step.as_ticks()));
        }
        assert_eq!(stepper.current_pos, Some(0));
    }

    #[test]
    fn test_move_travel_guards() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
        assert_eq!(
            stepper.planned_move(100).unwrap_err(),
            StepperError::NotHomed
        );
        let mut steps = stepper.homing_move(|| true);
        steps.next();
        assert_eq!(
            stepper.planned_move(TRAVEL_LIMIT.get() + 1).unwrap_err(),
            StepperError::MoveOutOfBounds
        );
    }

    #[test]
    fn test_move_max_vel() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
        let mut steps = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let (steps, _) = stepper.planned_move(TRAVEL_LIMIT.get()).unwrap();
        print!("speed,delay");
        for step in steps {
            println!("{},{}", (TICK_HZ / step.as_ticks()), step.as_ticks());
            assert!(step >= Duration::from_hz(MAX_VEL.get().into()));
        }
        assert_eq!(stepper.current_pos, Some(TRAVEL_LIMIT.get()));
    }

    #[test]
    fn test_move_max_accel() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
        let mut steps = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let mut prev_step = stepper.initial_delay;
        let mut time = Duration::from_ticks(0);

        let mut accels: [f64; _] = [0.0; 2];
        let mut accel_index = 0;

        let (steps, _) = stepper.planned_move(TRAVEL_LIMIT.get()).unwrap();
        println!("time,delay,vel,accel,avg_accel");
        for step in steps {
            let prev_vel = TICK_HZ as f64 / prev_step as f64;
            let vel = TICK_HZ as f64 / step.as_ticks() as f64;
            let accel = (vel - prev_vel) * prev_vel;
            accels[accel_index] = accel;
            accel_index = (accel_index + 1) % accels.len();
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
            // Also, for some reason there are single-step spikes, but they disappear when taking a
            // 2 step moving average.
            assert!(avg.abs() <= f64::from(MAX_ACCEL.get()) + (f64::from(MAX_ACCEL.get()) / 1.0));

            time += step;
            prev_step = step.as_ticks();
        }

        let final_vel = TICK_HZ as f64 / prev_step as f64;
        let final_accel = (f64::from(stepper.start_vel) - final_vel) * final_vel;
        accels[accel_index] = final_accel;
        let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
        println!(
            "{},{},{},{},{}",
            time.as_ticks(),
            prev_step,
            stepper.start_vel,
            final_accel,
            avg,
        );

        assert!(final_accel.abs() <= f64::from(MAX_ACCEL.get()) + 1.0);
        assert_eq!(stepper.current_pos, Some(TRAVEL_LIMIT.get()));
    }

    #[test]
    fn test_move_ends_at_start_vel() {
        // whatever the move distance, the last step has to be slow enough that stopping dead
        // from it stays within max_accel. MAX_VEL reaches its cruise speed well inside the
        // axis, FAST_VEL needs most of the axis to ramp, so between them the moves land on
        // both sides of the trapezoid/triangle split.
        const FAST_VEL: NonZeroU32 = NonZeroU32::new(400).unwrap();

        for max_vel in [MAX_VEL, FAST_VEL] {
            let mut stepper = Stepper::new(TRAVEL_LIMIT, max_vel, MAX_ACCEL, START_VEL);
            let mut steps = stepper.homing_move(|| true);
            steps.next();
            dbg!(&stepper);

            for distance in [1, 2, 3, 17, 64, 193, 500, 934, 1024, TRAVEL_LIMIT.get()] {
                let (steps, _) = stepper.planned_move(distance).unwrap();
                let final_delay = steps.last().unwrap();
                let final_vel = TICK_HZ as f64 / final_delay.as_ticks() as f64;
                let final_accel = (f64::from(START_VEL) - final_vel) * final_vel;
                println!("{distance} step move ended at {final_vel} steps/sec");
                assert!(
                    final_accel.abs() <= f64::from(MAX_ACCEL.get()) + 1.0,
                    "stopping a {distance} step move from {final_vel} steps/sec takes \
                    {final_accel} steps/sec^2"
                );

                // back home, ready for the next distance.
                let (steps, _) = stepper.planned_move(0).unwrap();
                steps.last();
            }
        }
    }

    #[test]
    fn test_move_max_accel_short() {
        let mut stepper = Stepper::new(TRAVEL_LIMIT, MAX_VEL, MAX_ACCEL, START_VEL);
        let mut steps = stepper.homing_move(|| true);
        steps.next();
        dbg!(&stepper);

        let mut prev_step = stepper.initial_delay;
        let mut time = Duration::from_ticks(0);

        let mut accels: [f64; _] = [0.0; 2];
        let mut accel_index = 0;

        let (steps, _) = stepper.planned_move(MAX_ACCEL.get()).unwrap();
        println!("time,delay,vel,accel,avg_accel");
        for step in steps {
            let prev_vel = TICK_HZ as f64 / prev_step as f64;
            let vel = TICK_HZ as f64 / step.as_ticks() as f64;
            let accel = (vel - prev_vel) * prev_vel;
            accels[accel_index] = accel;
            accel_index = (accel_index + 1) % accels.len();
            let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
            println!(
                "{},{},{},{},{}",
                time.as_ticks(),
                step.as_ticks(),
                vel,
                accel,
                avg,
            );

            // due to the fact we are using a first degree approximation of the ideal formula (which requires a square root), we sometimes go up 1% over our max acceleration.
            // Also, for some reason there are single-step spikes, but they disappear when taking a
            // 2 step moving average.
            assert!(avg.abs() <= f64::from(MAX_ACCEL.get()) + (f64::from(MAX_ACCEL.get()) / 1.0));

            time += step;
            prev_step = step.as_ticks();
        }

        let final_vel = TICK_HZ as f64 / prev_step as f64;
        let final_accel = (f64::from(stepper.start_vel) - final_vel) * final_vel;
        accels[accel_index] = final_accel;
        let avg: f64 = accels.iter().sum::<f64>() / accels.len() as f64;
        println!(
            "{},{},{},{},{}",
            time.as_ticks(),
            prev_step,
            stepper.start_vel,
            final_accel,
            avg,
        );

        assert!(final_accel.abs() <= f64::from(MAX_ACCEL.get()) + 1.0);
        assert_eq!(stepper.current_pos, Some(MAX_ACCEL.get()));
    }
}
