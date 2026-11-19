//! Background task that shows the odometry pose on the controller:
//!
//! ```text
//! line 2:  x 12.34 y -5.67     (fits 19 columns even at x/y = -123.45)
//! line 3:  h 90.12
//! ```
//!
//! x/y in inches, h = heading in degrees (0 = +Y, clockwise positive, -180..180). Line 1 is
//! left free for the auton PID error readout.

use vexide::controller::ControllerId;
use vexide::task;

use crate::config::TELEMETRY_PERIOD;
use crate::display::write_line;
use crate::localization::Localizer;
use crate::ticker::Ticker;

/// Start the readout. It runs for the whole program, in every competition mode.
pub fn start(loc: Localizer, controller: ControllerId) {
    task::spawn(run(loc, controller)).detach();
}

async fn run(loc: Localizer, controller: ControllerId) {
    let mut ticker = Ticker::new(TELEMETRY_PERIOD);
    // Alternate lines: the controller accepts about one write per tick, so writing both
    // lines each tick would just fail half the time.
    let mut show_heading = false;
    loop {
        let pose = loc.pose();
        let written = if show_heading {
            write_line(controller, 3, format_args!("h {:.2}", pose.theta.to_degrees()))
        } else {
            write_line(controller, 2, format_args!("x {:.2} y {:.2}", pose.x, pose.y))
        };
        // Busy (e.g. an auton motion just wrote line 1): retry the same line next tick.
        if written {
            show_heading = !show_heading;
        }
        ticker.tick().await;
    }
}
