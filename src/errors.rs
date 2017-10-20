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


/// The reason of the fail of inverse kinematics
#[derive(Debug)]
pub enum IKError {
    NotConverged,
    InverseMatrixError,
    PreconditionError,
    JointOutOfLimit(JointError),
}

impl From<JointError> for IKError {
    fn from(err: JointError) -> IKError {
        IKError::JointOutOfLimit(err)
    }
}

impl fmt::Display for IKError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            IKError::NotConverged => write!(f, "ik solve not converted"),
            IKError::InverseMatrixError => write!(f, "ik failed to solve inverse matrix"),
            IKError::PreconditionError => write!(f, "ik precondition not match"),
            IKError::JointOutOfLimit(ref err) => write!(f, "ik error : {}", err),
        }
    }
}

impl Error for IKError {
    fn description(&self) -> &str {
        match *self {
            IKError::NotConverged => "not converged",
            IKError::InverseMatrixError => "inverse matrix error",
            IKError::PreconditionError => "precondition not match",
            IKError::JointOutOfLimit(ref err) => err.description(),
        }
    }
}
