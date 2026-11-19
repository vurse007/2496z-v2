//! Controller-screen text without heap allocation: the auton PID error on line 1 and
//! (from `telemetry`) the pose on lines 2-3.

use std::fmt::{self, Write};
use std::time::Duration;

use vex_sdk::V5_ControllerId;
use vexide::controller::{ControllerConnection, ControllerId};
use vexide::prelude::Controller;
use vexide::time::sleep;

/// Pad every line to the full 19-column width so leftovers from longer text get wiped
/// (same as v1).
const PAD_TO: usize = Controller::MAX_COLUMNS;

/// Best-effort write of one formatted line (1-3) at column 1. Returns `false` if the
/// controller screen was busy (it takes about one write per 50 ms) or offline. It never waits.
pub fn write_line(id: ControllerId, line: u8, text: fmt::Arguments) -> bool {
    let mut buf = Line::default();
    let _ = buf.write_fmt(text);
    while buf.len < PAD_TO {
        let _ = buf.write_char(' ');
    }

    // Why not `Controller::try_set_text`? It builds a heap `CString` on every call, even when
    // the screen is busy, which means ~100 allocations/s inside motion loops. This is the same
    // SDK call with a stack buffer instead. Column 1 matches v1's `try_set_text(.., 1, 1)`.
    //
    // SAFETY: `buf.buf` is NUL-terminated (`Line` always keeps a trailing 0) and outlives the
    // call. The SDK copies the text before returning, the same thing vexide relies on when it
    // drops its `CString` right after this call.
    let id = V5_ControllerId::from(id);
    unsafe { vex_sdk::vexControllerTextSet(u32::from(id.0), u32::from(line), 1, buf.buf.as_ptr()) == 1 }
}

/// Best-effort write of `"<label> err: <error>"` to line 1 during auton motions.
/// Returns `false` only when the screen was busy.
pub fn show_error(controller: &Controller, label: &str, error: f64) -> bool {
    if controller.connection() == ControllerConnection::Offline {
        return true; // nothing to wait for, same as v1
    }
    write_line(controller.id(), 1, format_args!("{label} err: {error:.2}"))
}

/// Like `show_error`, but waits (non-blocking, 10 ms at a time) until the write goes through.
/// Used once per motion for the final settled error.
pub async fn show_final_error(controller: &Controller, label: &str, error: f64) {
    while !show_error(controller, label, error) {
        sleep(Duration::from_millis(10)).await;
    }
}

/// Fixed-capacity, NUL-terminated text buffer. Text that doesn't fit is cut off.
#[derive(Default)]
struct Line {
    buf: [u8; 32],
    len: usize,
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = self.buf.len() - 1 - self.len; // keep the last byte as the NUL terminator
        let n = s.len().min(room);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}
