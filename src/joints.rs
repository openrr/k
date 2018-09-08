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
use errors::*;
use na::{Isometry3, Real, Translation3, Unit, UnitQuaternion, Vector3};
use std::cell::RefCell;
use std::fmt::{self, Display};

/// Type of Joint, `Fixed`, `Rotational`, `Linear` is supported now
#[derive(Copy, Debug, Clone)]
pub enum JointType<T: Real> {
    /// Fixed joint. It has no `joint_angle` and axis.
    Fixed,
    /// Rotational joint around axis. It has an angle [rad].
    Rotational {
        /// axis of the joint
        axis: Unit<Vector3<T>>,
    },
    /// Linear joint. angle is length
    Linear {
        /// axis of the joint
        axis: Unit<Vector3<T>>,
    },
}

fn axis_to_string<T: Real>(axis: &Unit<Vector3<T>>) -> &str {
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

impl<T: Real> Display for JointType<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            JointType::Fixed => write!(f, "[⚓]"),
            JointType::Rotational { axis } => write!(f, "[⚙{}]", axis_to_string(axis)),
            JointType::Linear { axis } => write!(f, "[↕{}]", axis_to_string(axis)),
        }
    }
}

/// min/max range to check the joint position
#[derive(Copy, Debug, Clone)]
pub struct Range<T: Real> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
where
    T: Real,
{
    /// Create new Range instance
    ///
    /// # Examples
    ///
    /// ```
    /// let range = k::Range::new(-1.0, 1.0);
    /// ```
    pub fn new(min: T, max: T) -> Self {
        Range { min: min, max: max }
    }
    /// Check if the value is in the range
    ///
    /// `true` means it is OK.
    /// If the val is the same as the limit value (`min` or `max`), it returns true (valid).
    ///
    /// # Examples
    ///
    /// ```
    /// let range = k::Range::new(-1.0, 1.0);
    /// assert!(range.is_valid(0.0));
    /// assert!(range.is_valid(1.0));
    /// assert!(!range.is_valid(1.5));
    /// ```
    pub fn is_valid(&self, val: T) -> bool {
        val <= self.max && val >= self.min
    }
}

/// Information for copying joint state of other joint
///
/// For example, `Mimic` is used to calculate the angle of the gripper(R) from
/// gripper(L). In that case, the code like below will be used.
///
/// ```
/// let mimic_for_gripper_r = k::Mimic::new("gripper_l".to_owned(), -1.0, 0.0);
/// ```
///
/// output angle (mimic_angle() is calculated by `joint angles = joint[name] * multiplier + offset`
///
#[derive(Debug, Clone)]
pub struct Mimic<T: Real> {
    /// Name of the other joint
    pub name: String,
    pub multiplier: T,
    pub offset: T,
}

impl<T> Mimic<T>
where
    T: Real,
{
    /// Create new instance of Mimic
    ///
    /// # Examples
    ///
    /// ```
    /// let m = k::Mimic::<f64>::new("from".to_owned(), 1.0, 0.5);
    /// ```
    pub fn new(name: String, multiplier: T, offset: T) -> Self {
        Mimic {
            name: name,
            multiplier,
            offset,
        }
    }
    /// Calculate the mimic joint angle
    ///
    /// # Examples
    ///
    /// ```
    /// let m = k::Mimic::<f64>::new("from".to_owned(), 1.0, 0.5);
    /// assert_eq!(m.mimic_angle(0.2), 0.7); // 0.2 * 1.0 + 0.5
    /// ```
    ///
    /// ```
    /// let m = k::Mimic::<f64>::new("from".to_owned(), -2.0, -0.4);
    /// assert_eq!(m.mimic_angle(0.2), -0.8); // 0.2 * -2.0 - 0.4
    /// ```
    pub fn mimic_angle(&self, from_angle: T) -> T {
        from_angle * self.multiplier + self.offset
    }
}

/// Joint with type
#[derive(Debug, Clone)]
pub struct Joint<T: Real> {
    /// Name of this joint
    pub name: String,
    /// Type of this joint
    pub joint_type: JointType<T>,
    /// Angle(position) of this joint
    angle: T,
    /// Limits of this joint
    pub limits: Option<Range<T>>,
    /// local offset transform of joint
    pub offset: Isometry3<T>,
    /// cache of world transform
    world_transform_cache: RefCell<Option<Isometry3<T>>>,
}

impl<T> Joint<T>
where
    T: Real,
{
    /// Create new Joint with name and type
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// // create fixed joint
    /// let fixed = k::Joint::<f32>::new("f0", k::JointType::Fixed);
    /// assert!(fixed.angle().is_none());
    ///
    /// // create rotational joint with Y-axis
    /// let rot = k::Joint::<f64>::new("r0", k::JointType::Rotational { axis: na::Vector3::y_axis() });
    /// assert_eq!(rot.angle().unwrap(), 0.0);
    /// ```
    ///
    pub fn new(name: &str, joint_type: JointType<T>) -> Joint<T> {
        Joint {
            name: name.to_string(),
            joint_type: joint_type,
            angle: T::zero(),
            limits: None,
            offset: Isometry3::identity(),
            world_transform_cache: RefCell::new(None),
        }
    }
    /// Set the angle of the joint
    ///
    /// It returns Err if it is out of the limits, or this is fixed joint.
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// // Create fixed joint
    /// let mut fixed = k::Joint::<f32>::new("f0", k::JointType::Fixed);
    /// // Set angle to fixed joint always fails
    /// assert!(fixed.set_angle(1.0).is_err());
    ///
    /// // Create rotational joint with Y-axis
    /// let mut rot = k::Joint::<f64>::new("r0", k::JointType::Rotational { axis: na::Vector3::y_axis() });
    /// // As default, it has not limit
    ///
    /// // Initial angle is 0.0
    /// assert_eq!(rot.angle().unwrap(), 0.0);
    /// // If it has no limits, set_angle always succeeds.
    /// rot.set_angle(0.2).unwrap();
    /// assert_eq!(rot.angle().unwrap(), 0.2);
    /// ```
    ///
    pub fn set_angle(&mut self, angle: T) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimit {
                joint_name: self.name.to_string(),
                message: "Joint is Fixed".to_owned(),
            });
        }
        if let Some(range) = self.limits.clone() {
            if !range.is_valid(angle) {
                return Err(JointError::OutOfLimit {
                    joint_name: self.name.to_string(),
                    message: format!("Joint is out of range: input={}, range={:?}", angle, range),
                });
            }
        }
        self.angle = angle;
        // TODO: have to reset decendant `world_transform_cache`
        self.world_transform_cache.replace(None);
        Ok(())
    }
    /// Returns the angle (position)
    pub fn angle(&self) -> Option<T> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.angle),
        }
    }
    /// Calculate and returns the transform of the end of this joint
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// // Create linear joint with X-axis
    /// let mut lin = k::Joint::<f64>::new("l0", k::JointType::Linear { axis: na::Vector3::x_axis() });
    /// assert_eq!(lin.transform().translation.vector.x, 0.0);
    /// lin.set_angle(-1.0).unwrap();
    /// assert_eq!(lin.transform().translation.vector.x, -1.0);
    /// ```
    ///
    pub fn transform(&self) -> Isometry3<T> {
        let joint_transform = match self.joint_type {
            JointType::Fixed => Isometry3::identity(),
            JointType::Rotational { axis } => Isometry3::from_parts(
                Translation3::new(T::zero(), T::zero(), T::zero()),
                UnitQuaternion::from_axis_angle(&axis, self.angle),
            ),
            JointType::Linear { axis } => Isometry3::from_parts(
                Translation3::from_vector(axis.unwrap() * self.angle),
                UnitQuaternion::identity(),
            ),
        };
        self.offset * joint_transform
    }
    pub(crate) fn set_world_transform(&self, world_transform: Isometry3<T>) {
        self.world_transform_cache.replace(Some(world_transform));
    }
    /// Get the result of forward kinematics
    ///
    /// The value is updated by `LinkTree::update_transforms`
    pub fn world_transform(&self) -> Option<Isometry3<T>> {
        *self.world_transform_cache.borrow()
    }
}

impl<T: Real> Display for Joint<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} {}", self.name, self.joint_type,)
    }
}
