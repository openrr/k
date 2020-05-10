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
use nalgebra as na;
use thiserror::Error;

/// The reason of joint error
#[derive(Debug, Clone, Error)]
pub enum Error {
    /// Failed to set joint angle because the input is out of range or it is fixed joint
    #[error(
        "joint: {} is out of limit: {}, limit = {} <=> {}",
        joint_name,
        position,
        max_limit,
        min_limit
    )]
    OutOfLimitError {
        /// name of the joint
        joint_name: String,
        /// target position
        position: f64,
        /// max limit
        max_limit: f64,
        /// min limit
        min_limit: f64,
    },
    #[error("joint {} is fixed joint but the position is set", joint_name)]
    SetToFixedError {
        /// name of the joint
        joint_name: String,
    },
    /// Gave invalid size of vec as input
    #[error("size mismatch input = {}, required = {}", input, required)]
    SizeMismatchError {
        /// size of input
        input: usize,
        /// required size
        required: usize,
    },
    /// Error about mimic
    #[error("mimic error from {} to {}", from, to)]
    MimicError {
        /// tried to copy from `from`
        from: String,
        /// tried to copy to `to`
        to: String,
    },
    #[error(
        "ik solve not converged tried {} times, position diff = {}, rotation diff = {}",
        num_tried,
        position_diff,
        rotation_diff
    )]
    NotConvergedError {
        num_tried: usize,
        position_diff: na::Vector3<f64>,
        rotation_diff: na::Vector3<f64>,
    },
    #[error("inverse matrix error")]
    InverseMatrixError,
    #[error(
        "ik precondition error: input Dof={}, must be greater than {}",
        dof,
        necessary_dof
    )]
    PreconditionError { dof: usize, necessary_dof: usize },
}
