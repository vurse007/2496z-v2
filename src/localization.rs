//! Where the robot is. Today that's the IMU heading. This is also where odometry and
//! Monte Carlo Localization plug in, without touching motion code:
//!
//! - **Odometry:** add tracking-wheel `RotationSensor`s (they already implement `Encoder`)
//!   to `Localizer`, add an `update(&mut self)` that integrates encoder deltas + IMU heading
//!   into `pose`, and call it once per tick from the `Ticker` loops (driver and motions), or
//!   from a spawned 10 ms task. Then fill in `x`/`y` in `pose()`.
//! - **MCL:** keep the particles as a fixed-size array field on `Localizer` (no heap), use the
//!   odometry delta as the motion model and distance-sensor readings as the measurement model,
//!   and have `pose()` return the weighted mean.
//!
//! Motion code only calls `Localizer::pose()`, so none of it has to change.

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::{InertialSensor, RotationSensor};

use crate::config;

/// Anything that measures accumulated rotation in degrees: motor encoders, tracking wheels,
/// the lift sensor. `None` means unplugged or unreadable.
pub trait Encoder {
    fn position_deg(&self) -> Option<f64>;
}

impl Encoder for RotationSensor {
    // position() keeps counting past 360 (angle() would wrap), so errors never jump.
    fn position_deg(&self) -> Option<f64> {
        self.position().ok().map(|a| a.as_degrees())
    }
}

/// Field-relative robot pose.
#[derive(Clone, Copy)]
pub struct Pose {
    #[expect(dead_code, reason = "filled in by odometry; see module docs")]
    pub x: f64,
    #[expect(dead_code, reason = "filled in by odometry; see module docs")]
    pub y: f64,
    /// Compass heading in degrees, 0..360, clockwise positive (IMU convention).
    pub heading_deg: f64,
}

/// Owns the localization sensors and produces a `Pose`.
pub struct Localizer {
    imu: InertialSensor,
}

impl Localizer {
    /// Calibrates the IMU (~2-3 s, robot must be still). Call once before the match starts.
    pub async fn new(p: &mut DynamicPeripherals) -> Self {
        let port = p.take_smart_port(config::IMU_PORT).expect("smart port is already in use");
        let mut imu = InertialSensor::new(port);
        if let Err(err) = imu.calibrate().await {
            println!("IMU failed to calibrate: {err:?}");
        }
        // 5 ms updates, fresher than the 10 ms loop, for a smoother turn derivative.
        let _ = imu.set_data_interval(InertialSensor::MIN_DATA_INTERVAL);
        Self { imu }
    }

    /// Current best pose estimate, or `None` if the IMU is unplugged or still calibrating.
    pub fn pose(&self) -> Option<Pose> {
        let heading_deg = self.imu.heading().ok()?.as_degrees();
        Some(Pose { x: 0.0, y: 0.0, heading_deg })
    }
}
