/*
  Copyright 2020 Takashi Ogura

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
use nalgebra::{RealField, Unit, Vector3};
use std::fmt::{self, Display};

/// Type of Joint, `Fixed`, `Rotational`, `Linear` is supported now
#[derive(Copy, Debug, Clone)]
pub enum JointType<T: RealField> {
    /// Fixed joint. It has no `joint_position` and axis.
    Fixed,
    /// Rotational joint around axis. It has an position [rad].
    Rotational {
        /// axis of the joint
        axis: Unit<Vector3<T>>,
    },
    /// Linear joint. position is length
    Linear {
        /// axis of the joint
        axis: Unit<Vector3<T>>,
    },
}

fn axis_to_string<T: RealField>(axis: &Unit<Vector3<T>>) -> &str {
    if *axis == Vector3::x_axis() {
        "+X"
    } else if *axis == Vector3::y_axis() {
        "+Y"
    } else if *axis == Vector3::z_axis() {
        "+Z"
    } else if *axis == -Vector3::x_axis() {
        "-X"
    } else if *axis == -Vector3::y_axis() {
        "-Y"
    } else if *axis == -Vector3::z_axis() {
        "-Z"
    } else {
        ""
    }
}

impl<T: RealField> Display for JointType<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            JointType::Fixed => write!(f, "[⚓]"),
            JointType::Rotational { axis } => write!(f, "[⚙{}]", axis_to_string(axis)),
            JointType::Linear { axis } => write!(f, "[↕{}]", axis_to_string(axis)),
        }
    }
}
