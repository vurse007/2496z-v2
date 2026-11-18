//! Per-loop PID controller plus a settle detector. The math is identical to v1.

use std::time::{Duration, Instant};

/// Static PID settings. Every instance is a `const` in `config.rs`.
#[derive(Clone, Copy)]
pub struct PidConfig {
    /// `(kp, ki, kd)` used when |error| >= `refined_range`.
    pub general: (f64, f64, f64),
    /// `(kp, ki, kd)` used when |error| < `refined_range`.
    pub refined: (f64, f64, f64),
    pub refined_range: f64,
    /// |error| below this counts as "at target" for settling.
    pub settle_range: f64,
    /// Max change in output volts per loop, both directions.
    pub slew: f64,
    /// Integral only accumulates while |error| is below this.
    pub integral_threshold: f64,
    /// Clamp on the summed error (error units, not volts).
    pub max_integral: f64,
    /// |error| below this is treated as zero for P and I.
    pub deadband: f64,
    /// How long the error has to stay inside `settle_range` before a motion ends.
    pub settle_ms: u64,
}

pub struct Pid {
    cfg: PidConfig,
    /// Error after the deadband. Shown on the controller and used by the lift's settle check.
    error: f64,
    /// Raw (pre-deadband) error from the last loop, so the derivative is smooth.
    prev_error: f64,
    integral: f64,
    prev_output: f64,
}

impl Pid {
    pub const fn new(cfg: PidConfig) -> Self {
        Self { cfg, error: 0.0, prev_error: 0.0, integral: 0.0, prev_output: 0.0 }
    }

    pub const fn config(&self) -> &PidConfig {
        &self.cfg
    }

    /// Error after the deadband from the last `calculate()`.
    pub const fn error(&self) -> f64 {
        self.error
    }

    pub fn reset(&mut self) {
        self.error = 0.0;
        self.prev_error = 0.0;
        self.integral = 0.0;
        self.prev_output = 0.0;
    }

    /// One PID step. Returns volts, limited to `12 * scale` (scale 0.0..=1.0 is a speed cap).
    pub fn calculate(&mut self, target: f64, current: f64, scale: f64) -> f64 {
        let c = &self.cfg;
        let raw_error = target - current;

        // Deadband: tiny error counts as zero for P and I.
        let error = if raw_error.abs() < c.deadband { 0.0 } else { raw_error };

        // Trapezoidal integral, clamped, frozen (not reset) outside the threshold.
        if error.abs() < c.integral_threshold {
            self.integral = (self.integral + (error + self.prev_error) * 0.5)
                .clamp(-c.max_integral, c.max_integral);
        }

        // Derivative uses the raw error so it tapers smoothly into the deadband.
        let derivative = raw_error - self.prev_error;

        let (kp, ki, kd) = if error.abs() < c.refined_range { c.refined } else { c.general };
        let raw_output = kp * error + ki * self.integral + kd * derivative;

        // Slew, then the speed cap.
        let max_output = 12.0 * scale.clamp(0.0, 1.0);
        let output = raw_output
            .clamp(self.prev_output - c.slew, self.prev_output + c.slew)
            .clamp(-max_output, max_output);

        self.error = error;
        self.prev_error = raw_error;
        self.prev_output = output;
        output
    }
}

/// Settle detector: done once |error| has stayed inside the range for longer than the hold time.
pub struct Settle {
    range: f64,
    /// v1 checked `elapsed_ms > settle_ms` with truncated milliseconds. That's the same as
    /// `elapsed >= settle_ms + 1 ms`, which is precomputed here so the loop only does a compare.
    hold: Duration,
    since: Option<Instant>,
}

impl Settle {
    pub const fn new(cfg: &PidConfig) -> Self {
        Self { range: cfg.settle_range, hold: Duration::from_millis(cfg.settle_ms + 1), since: None }
    }

    /// Feed one loop's |error|. Returns true once settled.
    pub fn update(&mut self, abs_error: f64, now: Instant) -> bool {
        if abs_error < self.range {
            let since = *self.since.get_or_insert(now);
            now - since >= self.hold
        } else {
            self.since = None;
            false
        }
    }
}
