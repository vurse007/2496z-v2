//! The robot: owns every subsystem, runs driver control, and dispatches auton.

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;

use crate::config::{self, LOOP_PERIOD};
use crate::drive::Drivetrain;
use crate::lift::Lift;
use crate::localization::Localizer;
use crate::motor_group::MotorGroup;
use crate::telemetry;
use crate::ticker::Ticker;

pub struct Robot {
    pub drive: Drivetrain,
    pub lift: Lift,
    pub claw: MotorGroup<1>,
    pub toggles: MotorGroup<2>,
    pub flipper: AdiDigitalOut,
    pub park: AdiDigitalOut,
    pub controller: Controller,
    /// Handle to the background odometry task (it keeps running whether or not this is read).
    #[expect(dead_code, reason = "auton motion stack; used again once autons come back")]
    pub loc: Localizer,
}

impl Robot {
    /// Builds the drivetrain, then calibrates the IMU (~2-3 s, robot must be still) and starts
    /// background odometry and the controller pose readout, then builds everything else.
    /// Panics at startup if two devices share a port.
    pub async fn new(p: &mut DynamicPeripherals) -> Self {
        let drive = Drivetrain::new(p);
        let loc = Localizer::start(p, drive.encoders()).await;
        let controller = p.take_primary_controller().expect("primary controller already taken");
        telemetry::start(loc.clone(), controller.id()); // pose on controller lines 2-3
        Self {
            drive,
            lift: Lift::new(p),
            claw: MotorGroup::new(p, config::CLAW, config::CLAW_GEARSET, config::CLAW_BRAKE),
            toggles: MotorGroup::new(
                p,
                config::TOGGLE_MOTORS,
                config::TOGGLE_GEARSET,
                config::TOGGLE_BRAKE,
            ),
            flipper: AdiDigitalOut::new(adi_port(p, config::FLIPPER_ADI_PORT)),
            park: AdiDigitalOut::new(adi_port(p, config::PARK_ADI_PORT)),
            controller,
            loc,
        }
    }

    /// Re-send every motor command on the next write. Called on entering each competition mode.
    fn resync_outputs(&mut self) {
        self.drive.resync();
        self.lift.motors.resync();
        self.claw.resync();
        self.toggles.resync();
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        self.resync_outputs();
        // No routines yet. They'll come back with the auton selector.
    }

    async fn driver(&mut self) {
        println!("Driver!");
        self.resync_outputs();
        let mut ticker = Ticker::new(LOOP_PERIOD);

        loop {
            // Controller disconnected -> all zeros/released -> robot stops.
            let s = self.controller.state().unwrap_or_default();

            // Split arcade: left stick Y = forward, right stick X = turn.
            self.drive.split_arcade(s.left_stick.y_raw(), s.right_stick.x_raw());

            // Lift. In v1 the R1 (claw) block ran last and overwrote the lift, so the effective
            // priority is R1 > L1 > L2. One write per loop instead of two.
            if s.button_r1.is_pressed() {
                self.lift.motors.set_voltage(config::LIFT_DOWN_WITH_CLAW_V);
            } else if s.button_l1.is_pressed() {
                self.lift.motors.set_voltage(config::LIFT_UP_V);
            } else if s.button_l2.is_pressed() {
                self.lift.motors.set_voltage(config::LIFT_DOWN_V);
            } else {
                self.lift.motors.brake();
            }

            // Claw: R1 opens, R2 closes, otherwise 0 V.
            if s.button_r1.is_pressed() {
                self.claw.set_voltage(config::CLAW_OPEN_V);
            } else if s.button_r2.is_pressed() {
                self.claw.set_voltage(config::CLAW_CLOSE_V);
            } else {
                self.claw.set_voltage(0.0);
            }

            // Toggles: hold Y.
            let toggles_v = if s.button_y.is_pressed() { config::TOGGLES_V } else { 0.0 };
            self.toggles.set_voltage(toggles_v);

            // Pistons: D-pad left / right toggle on press.
            if s.button_left.is_now_pressed() {
                let _ = self.flipper.toggle();
            }
            if s.button_right.is_now_pressed() {
                let _ = self.park.toggle();
            }

            ticker.tick().await;
        }
    }
}

fn adi_port(p: &mut DynamicPeripherals, port: u8) -> vexide::adi::AdiPort {
    p.take_adi_port(port).expect("adi port is already in use")
}
