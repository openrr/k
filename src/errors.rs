/*
  Copyright 2017 Takashi Ogura

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

/// The reason of joint error
#[derive(Debug, Clone, Fail)]
pub enum JointError {
    /// Failed to set joint angle because the input is out of range or it is fixed joint
    #[fail(display = "joint: {} is out of limit: {}", joint_name, message)]
    OutOfLimitError {
        /// name of the joint
        joint_name: String,
        /// detail error message
        message: String,
    },
    /// Gave invalid size of vec as input
    #[fail(display = "size mismatch input = {}, required = {}", input, required)]
    SizeMismatchError {
        /// size of input
        input: usize,
        /// required size
        required: usize,
    },
    /// Error about mimic
    #[fail(display = "mimic error from {} to {}", from, to)]
    MimicError {
        /// tried to copy from `from`
        from: String,
        /// tried to copy to `to`
        to: String,
        /// detail error message
        message: String,
    },
    #[fail(display = "invalid arguments {:?}", error)]
    InvalidArgumentsError { error: String },
}

/// The reason of the fail of inverse kinematics
#[derive(Debug, Fail)]
pub enum IKError {
    #[fail(display = "ik solve not converged {:?}", error)]
    NotConvergedError { error: String },
    #[fail(display = "inverse matrix error")]
    InverseMatrixError,
    #[fail(display = "ik precondition error {:?}", error)]
    PreconditionError { error: String },
    #[fail(display = "joint error: {:?}", error)]
    JointOutOfLimitError { error: JointError },
    #[fail(display = "invalid arguments {:?}", error)]
    InvalidArgumentsError { error: String },
}

impl From<JointError> for IKError {
    fn from(error: JointError) -> IKError {
        IKError::JointOutOfLimitError { error }
    }
}
