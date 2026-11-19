//! Arc-corrected tracking-wheel odometry, ported from rampage-v3 `lynx::odom_drive::update`.
//!
//! Pure math: it takes one `Reading` per tick and never touches hardware, so it doesn't care
//! whether distances come from tracking wheels or drive-motor encoders.
//!
//! Frame (same as rampage-v3, and what its mtp/chase motions assume):
//!   x, y in inches. theta in radians, 0 = +Y ("north"), clockwise positive (the IMU's
//!   convention), always wrapped to [-pi, pi).

use std::f64::consts::{PI, TAU};

use super::Pose;

/// One tick of raw sensor data, already converted to inches/degrees.
/// `None` = no sensor or unreadable this tick. That axis then contributes no motion this
/// tick, and the next good reading picks up the full distance, so nothing is lost.
pub struct Reading {
    /// Forward travel: vertical tracking wheel, or averaged drive motors as a fallback.
    pub vertical_in: Option<f64>,
    /// Sideways travel (right = positive): horizontal tracking wheel.
    pub horizontal_in: Option<f64>,
    /// Raw IMU compass heading, 0..360.
    pub imu_heading_deg: Option<f64>,
}

pub struct Odometry {
    pose: Pose,
    /// IMU reading that corresponds to theta = 0, kept in [0, 360) (see `reset`).
    heading_offset_deg: f64,
    prev_vertical_in: f64,
    prev_horizontal_in: f64,
    /// Distance of each wheel from the tracking center, used to cancel the arc a wheel
    /// sweeps when the robot turns in place. 0 for a missing wheel or the motor fallback.
    vertical_offset_in: f64,
    horizontal_offset_in: f64,
}

impl Odometry {
    pub const fn new(vertical_offset_in: f64, horizontal_offset_in: f64) -> Self {
        Self {
            pose: Pose::ORIGIN,
            heading_offset_deg: 0.0,
            prev_vertical_in: 0.0,
            prev_horizontal_in: 0.0,
            vertical_offset_in,
            horizontal_offset_in,
        }
    }

    pub const fn pose(&self) -> Pose {
        self.pose
    }

    /// Jump to `pose`. Instead of zeroing the encoders like rampage-v3 does, this latches the
    /// current readings as the new baseline. The math only uses deltas, so the result is the
    /// same, with no extra SDK writes and no reset side effects for anything else reading
    /// those encoders.
    pub fn reset(&mut self, pose: Pose, r: &Reading) {
        // Non-finite theta would never wrap (and would poison the pose), so fall back to 0.
        let theta = if pose.theta.is_finite() { wrap_to_pi(pose.theta) } else { 0.0 };
        self.pose = Pose { theta, ..pose };

        if let Some(imu) = r.imu_heading_deg.filter(|d| d.is_finite()) {
            // Pick the offset so that (imu - offset) maps to the requested theta.
            self.heading_offset_deg = (imu - theta.to_degrees()).rem_euclid(360.0);
        }
        if let Some(v) = r.vertical_in {
            self.prev_vertical_in = v;
        }
        if let Some(h) = r.horizontal_in {
            self.prev_horizontal_in = h;
        }
    }

    /// Advance the pose by one tick of sensor data.
    pub fn update(&mut self, r: &Reading) {
        // --- heading: absolute from the IMU, so it never drifts with integration ---
        // heading in [0, 360) and offset in [0, 360) -> difference in (-360, 360), so
        // `wrap_to_pi` loops at most once (no fmod).
        let theta = match r.imu_heading_deg {
            Some(deg) if deg.is_finite() => {
                wrap_to_pi((deg - self.heading_offset_deg).to_radians())
            }
            _ => self.pose.theta, // IMU unreadable: hold heading, keep integrating distance
        };
        let d_theta = wrap_to_pi(theta - self.pose.theta);
        let avg_theta = self.pose.theta + 0.5 * d_theta;

        // --- encoder deltas ---
        let d_vertical = take_delta(&mut self.prev_vertical_in, r.vertical_in);
        let d_horizontal = take_delta(&mut self.prev_horizontal_in, r.horizontal_in);

        // --- arc-corrected displacement in the robot frame ---
        // rampage-v3: local = 2*sin(dθ/2) * (d/dθ - offset), with a separate dθ ≈ 0 branch.
        // Rewritten as chord_factor(dθ) * (d - offset*dθ): same value, one formula, and no
        // division or sin for normal-size steps (see `chord_factor`).
        let chord = chord_factor(d_theta);
        let local_x = chord * (d_horizontal - self.horizontal_offset_in * d_theta);
        let local_y = chord * (d_vertical - self.vertical_offset_in * d_theta);

        // --- rotate into the field frame at the average heading over the step ---
        // One `sin_cos` call is this function's only trig.
        let (sin_h, cos_h) = avg_theta.sin_cos();
        self.pose.x += local_x * cos_h + local_y * sin_h;
        self.pose.y += local_y * cos_h - local_x * sin_h;
        self.pose.theta = theta;
    }
}

/// Distance since the last good reading. Keeps the old baseline when the read failed.
fn take_delta(prev: &mut f64, now: Option<f64>) -> f64 {
    match now {
        Some(now) => {
            let d = now - *prev;
            *prev = now;
            d
        }
        None => 0.0,
    }
}

/// `2·sin(dθ/2) / dθ`: the ratio of chord length to arc length for a turn of `dθ`.
///
/// At 10 ms a step is far below 0.25 rad (that would be over 1400 deg/s), so the Taylor series
/// `1 - dθ²/24 + dθ⁴/1920` is what normally runs: three multiplies and two adds, no sin, no
/// division, and no special case at 0. Its error is below dθ⁶/322560, which is under 1e-9
/// even at the 0.25 cutoff. Larger steps (e.g. after a long gap between updates) use the
/// exact formula.
fn chord_factor(d_theta: f64) -> f64 {
    if d_theta.abs() < 0.25 {
        let d2 = d_theta * d_theta;
        1.0 - d2 * (1.0 / 24.0) + d2 * d2 * (1.0 / 1920.0)
    } else {
        2.0 * (0.5 * d_theta).sin() / d_theta
    }
}

/// Wrap to [-pi, pi). Inputs here are within a couple of turns, so a compare and an add
/// replace rampage-v3's `fmod`. `a` must be finite; every caller checks this.
fn wrap_to_pi(mut a: f64) -> f64 {
    while a >= PI {
        a -= TAU;
    }
    while a < -PI {
        a += TAU;
    }
    a
}
