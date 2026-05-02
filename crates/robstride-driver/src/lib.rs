//! Linux SocketCAN driver for the Robstride CAN servo motor family.
//!
//! ```no_run
//! use robstride_driver::{Motor, MotorModel, RunMode};
//!
//! let mut motor = Motor::open("can0", 1, MotorModel::Rs05)?;
//! motor.enable()?;
//! motor.set_run_mode(RunMode::Position)?;
//! motor.set_position(3.14)?;
//! let fb = motor.read_status()?;
//! println!("position = {:.3} rad", fb.position);
//! motor.disable()?;
//! # Ok::<(), robstride_driver::Error>(())
//! ```

pub mod driver;
pub mod error;
pub mod scan;

pub use driver::Motor;
pub use error::{Error, Result};
pub use scan::{ScanProgress, ScanResult, dump_bus, scan_bus};

/// Re-export of the protocol crate so consumers can drop down to raw frames.
pub use robstride_protocol as protocol;

pub use robstride_protocol::{
    DEFAULT_HOST_ID, MitScales, MotorFeedback, MotorModel, MotorStatusBits, ParamIndex, RunMode,
};
