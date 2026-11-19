//! Auton motions: drive straight, point turn, swing turn. The behavior matches v1, but each
//! loop runs on a fixed 10 ms grid and never allocates.

use std::time::{Duration, Instant};

use vexide::prelude::Controller;

use crate::config;
use crate::display::{show_error, show_final_error};
use crate::drive::Drivetrain;
use crate::localization::{Encoder, Localizer};
use crate::pid::Settle;
use crate::ticker::Ticker;

/// Which side drives during a turn. `Both` = point turn. `Left`/`Right` = swing turn, where
/// only that side spins (a Left swing pivots around the right wheels).
#[derive(Clone, Copy)]
pub enum SwingSide {
    Both,
    Left,
    Right,
}

/// Safety timeout for a motion. The deadline is precomputed, so each check is one compare.
pub struct MotionTimer {
    deadline: Instant,
}

impl MotionTimer {
    pub fn start(timeout_ms: u64) -> Self {
        Self { deadline: Instant::now() + Duration::from_millis(timeout_ms) }
    }

    pub fn expired(&self, now: Instant) -> bool {
        now >= self.deadline
    }
}

/// Shortest signed angle from `current` to `target`, in [-180, 180).
///
/// v1 used `(e + 180).rem_euclid(360) - 180`, which is a software fmod on the V5's CPU. Inputs
/// here are a heading in [0, 360) and a target within a turn or two of it, so one or two
/// add/subtracts do the same job.
fn wrap_degrees(target: f64, current: f64) -> f64 {
    let mut e = target - current;
    while e >= 180.0 {
        e -= 360.0;
    }
    while e < -180.0 {
        e += 360.0;
    }
    e
}

impl Drivetrain {
    /// Turn to an absolute IMU heading (degrees). `side` picks point turn vs swing.
    pub async fn turn_to(
        &mut self,
        loc: &Localizer,
        controller: &Controller,
        side: SwingSide,
        target_heading: f64,
        timeout_ms: u64,
        scale: f64,
    ) {
        self.turn_pid.reset();
        let mut settle = Settle::new(self.turn_pid.config());
        let timer = MotionTimer::start(timeout_ms);
        let mut ticker = Ticker::new(config::LOOP_PERIOD);

        loop {
            let Some(heading) = loc.heading_deg() else {
                break; // IMU unplugged or calibrating: bail instead of spinning forever
            };
            let error = wrap_degrees(target_heading, heading);
            let speed = self.turn_pid.calculate(error, 0.0, scale);

            match side {
                SwingSide::Both => self.move_raw(speed, -speed),
                SwingSide::Left => self.move_raw(speed, 0.0),
                SwingSide::Right => self.move_raw(0.0, -speed),
            }
            show_error(controller, "turn", error);

            let now = Instant::now();
            if settle.update(error.abs(), now) || timer.expired(now) {
                break;
            }
            ticker.tick().await;
        }

        self.target_heading = target_heading;
        self.stop();

        let final_error = loc
            .heading_deg()
            .map_or(self.turn_pid.error(), |h| wrap_degrees(target_heading, h));
        show_final_error(controller, "turn", final_error).await;
    }

    /// Drive `distance` motor-encoder degrees (negative = backward) while holding
    /// `target_heading` from the last turn.
    pub async fn straight(
        &mut self,
        loc: &Localizer,
        controller: &Controller,
        distance: f64,
        timeout_ms: u64,
        scale: f64,
    ) {
        self.drive_pid.reset();
        self.heading_pid.reset();

        let Some(start) = self.position_deg() else {
            return;
        };
        let mut settle = Settle::new(self.drive_pid.config());
        let timer = MotionTimer::start(timeout_ms);
        let mut ticker = Ticker::new(config::LOOP_PERIOD);

        loop {
            let Some(current) = self.position_deg() else {
                break;
            };
            let traveled = current - start;

            let drive = self.drive_pid.calculate(distance, traveled, scale);
            show_error(controller, "drive", self.drive_pid.error());

            // Heading hold. With no IMU, skip the correction and just drive.
            let heading_error =
                loc.heading_deg().map_or(0.0, |h| wrap_degrees(self.target_heading, h));
            let correction = self.heading_pid.calculate(heading_error, 0.0, 1.0);

            // Leave room for the correction. Otherwise at full speed one side clamps at 12 V
            // and the correction is lost.
            let max_drive = 12.0 - correction.abs();
            let drive = drive.clamp(-max_drive, max_drive);
            self.move_raw(drive + correction, drive - correction);

            let now = Instant::now();
            if settle.update((distance - traveled).abs(), now) || timer.expired(now) {
                break;
            }
            ticker.tick().await;
        }
        self.stop();

        let final_error = self
            .position_deg()
            .map_or(self.drive_pid.error(), |p| distance - (p - start));
        show_final_error(controller, "drive", final_error).await;
    }
}
