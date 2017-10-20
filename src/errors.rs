use std::error::Error;
use std::fmt;

/// The reason of joint error: `OutOfLimit`, `SizeMisMatch`
#[derive(Debug, Clone)]
pub enum JointError {
    OutOfLimit,
    SizeMisMatch,
}

impl fmt::Display for JointError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            JointError::OutOfLimit => write!(f, "limit over"),
            JointError::SizeMisMatch => write!(f, "size is invalid"),
        }
    }
}

impl Error for JointError {
    fn description(&self) -> &str {
        match *self {
            JointError::OutOfLimit => "limit over",
            JointError::SizeMisMatch => "size is invalid",
        }
    }
}
