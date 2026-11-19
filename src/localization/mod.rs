//! Where the robot is: IMU heading plus tracking-wheel odometry.
//!
//! The localization sensors (IMU, tracking wheels) live in a `Tracker` that a background task
//! owns and updates every 10 ms, no matter what auton or driver code is doing (sleeps, lift
//! moves, controller waits). It doesn't own the drive motors. When there's no vertical
//! tracking wheel, it reads them through a shared `Encoder` handle (see `drive::DriveMotors`).
//! `Localizer` is the handle the rest of the code uses to read heading/pose or reset.
//!
//! Wheel setups (each axis is picked independently in `config.rs`):
//!
//! | vertical pod | horizontal pod | rampage-v3 name | forward from        | sideways from |
//! |--------------|----------------|-----------------|---------------------|---------------|
//! | yes          | yes            | DUAL            | vertical pod        | horizontal pod|
//! | yes          | no             | VERT            | vertical pod        | none (0)      |
//! | no           | yes            | HORIZ           | drive motors        | horizontal pod|
//! | no           | no             | (not in v3)     | drive motors        | none (0)      |
//!
//! **Update rate:** a fixed 10 ms `Ticker` grid in `run()`, started once at boot by
//! `Localizer::start`. Spawning allocates the task once at startup; the loop itself never
//! allocates.
//!
//! **MCL plugs in here.** Each tick, `odom.update()` produces the robot-frame step
//! (`local_x`, `local_y`, `d_theta` in `odom.rs`). That's the motion model input. Add a
//! fixed-size particle array to `Tracker` (no heap), apply that step plus noise to each
//! particle in `Tracker::update()`, weight the particles with distance-sensor readings, and
//! have `pose()` return the weighted estimate. It already runs in the background task, so
//! callers keep using `Localizer::pose()` unchanged.

mod odom;

use std::cell::RefCell;
use std::rc::Rc;

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::{InertialSensor, RotationSensor};
use vexide::task;

use crate::config::{self, LOOP_PERIOD, PodConfig};
use crate::ticker::Ticker;
use odom::{Odometry, Reading};

/// Anything that measures accumulated rotation in degrees: motor encoders, tracking wheels,
/// the lift sensor, the drivetrain as a whole. `None` means unplugged or unreadable.
pub trait Encoder {
    fn position_deg(&self) -> Option<f64>;
}

/// Shared encoders (the drive motors) are read through a `RefCell`. `try_borrow` means a read
/// that somehow collided with a write just counts as "unreadable this tick". Odometry then
/// picks up the full distance on the next tick, so nothing is lost and nothing can panic.
impl<T: Encoder> Encoder for RefCell<T> {
    fn position_deg(&self) -> Option<f64> {
        self.try_borrow().ok()?.position_deg()
    }
}

impl Encoder for RotationSensor {
    // position() keeps counting past 360 (angle() would wrap), so errors never jump.
    fn position_deg(&self) -> Option<f64> {
        self.position().ok().map(|a| a.as_degrees())
    }
}

/// Field-relative robot pose: x, y in inches; theta in radians, 0 = +Y, clockwise positive,
/// wrapped to [-pi, pi). The same frame as rampage-v3, so its mtp/chase math ports directly.
#[derive(Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose {
    pub const ORIGIN: Self = Self { x: 0.0, y: 0.0, theta: 0.0 };
}

/// A tracking wheel: rotation sensor + precomputed inches-per-degree.
struct TrackingWheel {
    sensor: RotationSensor,
    inches_per_deg: f64,
}

impl TrackingWheel {
    fn new(p: &mut DynamicPeripherals, cfg: PodConfig) -> Self {
        let port = p.take_smart_port(cfg.port).expect("smart port is already in use");
        let mut sensor = RotationSensor::new(port, cfg.direction);
        // 5 ms updates, so every 10 ms tick sees a fresh reading.
        let _ = sensor.set_data_interval(RotationSensor::MIN_DATA_INTERVAL);
        Self { sensor, inches_per_deg: config::inches_per_deg(cfg.wheel_diameter_in) }
    }

    fn inches(&self) -> Option<f64> {
        Some(self.sensor.position_deg()? * self.inches_per_deg)
    }
}

/// Everything the background odometry task owns.
struct Tracker {
    imu: InertialSensor,
    vertical: Option<TrackingWheel>,
    horizontal: Option<TrackingWheel>,
    /// Drive-motor encoders. Only read when there's no vertical pod.
    drive: Rc<dyn Encoder>,
    odom: Odometry,
}

impl Tracker {
    /// Raw IMU compass heading in degrees, 0..360, or `None` if unplugged/calibrating.
    fn heading_deg(&self) -> Option<f64> {
        self.imu.heading().ok().map(|a| a.as_degrees())
    }

    fn update(&mut self) {
        let reading = self.read();
        self.odom.update(&reading);
    }

    fn reset(&mut self, pose: Pose) {
        let reading = self.read();
        self.odom.reset(pose, &reading);
    }

    fn read(&self) -> Reading {
        let vertical_in = match &self.vertical {
            Some(wheel) => wheel.inches(),
            None => self.drive.position_deg().map(|deg| deg * config::DRIVE_IN_PER_MOTOR_DEG),
        };
        Reading {
            vertical_in,
            horizontal_in: self.horizontal.as_ref().and_then(TrackingWheel::inches),
            imu_heading_deg: self.heading_deg(),
        }
    }
}

/// The background task: one odometry step every 10 ms, forever (all competition modes,
/// including disabled, so the pose survives the robot being pushed between modes).
async fn run(tracker: Rc<RefCell<Tracker>>) {
    let mut ticker = Ticker::new(LOOP_PERIOD);
    loop {
        tracker.borrow_mut().update(); // borrow ends here, before the await
        ticker.tick().await;
    }
}

/// Handle to the running odometry. Cloning it is cheap (an `Rc` clone), and each clone sees
/// the same pose. Every method borrows the tracker briefly and never across
/// an `.await`, so it can't collide with the background task (see `drive::DriveMotors`).
#[derive(Clone)]
pub struct Localizer {
    tracker: Rc<RefCell<Tracker>>,
}

impl Localizer {
    /// Calibrates the IMU (~2-3 s, robot must be still), sets the pose to the origin, and
    /// starts the background odometry task. Call once at boot. `drive` is the drive-motor
    /// encoder, used only as the fallback when there's no vertical tracking wheel.
    pub async fn start(p: &mut DynamicPeripherals, drive: Rc<dyn Encoder>) -> Self {
        let port = p.take_smart_port(config::IMU_PORT).expect("smart port is already in use");
        let mut imu = InertialSensor::new(port);
        if let Err(err) = imu.calibrate().await {
            println!("IMU failed to calibrate: {err:?}");
        }
        // 5 ms updates, fresher than the 10 ms loop, for a smoother turn derivative.
        let _ = imu.set_data_interval(InertialSensor::MIN_DATA_INTERVAL);

        let vertical = config::VERTICAL_POD.map(|cfg| TrackingWheel::new(p, cfg));
        let horizontal = config::HORIZONTAL_POD.map(|cfg| TrackingWheel::new(p, cfg));

        // A missing wheel (or the motor fallback, which measures the track center) has no
        // offset to correct for.
        let offset = |cfg: Option<PodConfig>| cfg.map_or(0.0, |c| c.offset_in);
        let odom = Odometry::new(offset(config::VERTICAL_POD), offset(config::HORIZONTAL_POD));

        let mut tracker = Tracker { imu, vertical, horizontal, drive, odom };
        tracker.reset(Pose::ORIGIN); // baseline the encoders; autons set their own start pose

        let tracker = Rc::new(RefCell::new(tracker));
        task::spawn(run(Rc::clone(&tracker))).detach();
        Self { tracker }
    }

    /// Raw IMU compass heading in degrees, 0..360, or `None` if unplugged/calibrating.
    /// The v1 turn/straight motions use this directly, so their behavior is unchanged
    /// regardless of any odometry `reset()`.
    pub fn heading_deg(&self) -> Option<f64> {
        self.tracker.borrow().heading_deg()
    }

    /// Current best pose estimate. MCL replaces what this returns; callers don't change.
    pub fn pose(&self) -> Pose {
        self.tracker.borrow().odom.pose()
    }
}

#[expect(dead_code, reason = "used by autons, MCL and pose-based motions (mtp/chase) once they land")]
impl Localizer {

    /// Set the pose, e.g. the starting position of an auton. Heading is matched by storing an
    /// IMU offset, so the IMU itself is never re-zeroed.
    pub fn reset(&self, pose: Pose) {
        self.tracker.borrow_mut().reset(pose);
    }
}
