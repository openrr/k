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
//! Joint related structs
use errors::*;
use na::{Isometry3, Real, Translation3, Unit, UnitQuaternion, Vector3};
use std::cell::RefCell;
use std::fmt::{self, Display};

#[derive(Clone, Debug, Copy)]
pub struct Velocity<T: Real> {
    pub translation: Vector3<T>,
    pub rotation: Vector3<T>,
}

impl<T> Velocity<T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self::zero()
    }
    pub fn from_parts(translation: Vector3<T>, rotation: Vector3<T>) -> Self {
        Self {
            translation,
            rotation,
        }
    }
    pub fn zero() -> Self {
        Self {
            translation: Vector3::zeros(),
            rotation: Vector3::zeros(),
        }
    }
}

impl<T> Default for Velocity<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Type of Joint, `Fixed`, `Rotational`, `Linear` is supported now
#[derive(Copy, Debug, Clone)]
pub enum JointType<T: Real> {
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
    /// let range = k::joint::Range::new(-1.0, 1.0);
    /// ```
    pub fn new(min: T, max: T) -> Self {
        Range { min, max }
    }
    /// Check if the value is in the range
    ///
    /// `true` means it is OK.
    /// If the val is the same as the limit value (`min` or `max`), it returns true (valid).
    ///
    /// # Examples
    ///
    /// ```
    /// let range = k::joint::Range::new(-1.0, 1.0);
    /// assert!(range.is_valid(0.0));
    /// assert!(range.is_valid(1.0));
    /// assert!(!range.is_valid(1.5));
    /// ```
    pub fn is_valid(&self, val: T) -> bool {
        val <= self.max && val >= self.min
    }
}

impl<T> From<::std::ops::RangeInclusive<T>> for Range<T>
where
    T: Real,
{
    /// # Examples
    ///
    /// ```
    /// let range : k::joint::Range<f64> = (-1.0..=1.0).into();
    /// assert!(range.is_valid(0.0));
    /// assert!(range.is_valid(1.0));
    /// assert!(!range.is_valid(1.5));
    /// ```
    fn from(range: ::std::ops::RangeInclusive<T>) -> Self {
        let (min, max) = range.into_inner();
        Range { min, max }
    }
}

/// Information for copying joint state of other joint
///
/// For example, `Mimic` is used to calculate the position of the gripper(R) from
/// gripper(L). In that case, the code like below will be used.
///
/// ```
/// let mimic_for_gripper_r = k::joint::Mimic::new(-1.0, 0.0);
/// ```
///
/// output position (mimic_position() is calculated by `joint positions = joint[name] * multiplier + origin`
///
#[derive(Debug, Clone)]
pub struct Mimic<T: Real> {
    pub multiplier: T,
    pub origin: T,
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
    /// let m = k::joint::Mimic::<f64>::new(1.0, 0.5);
    /// ```
    pub fn new(multiplier: T, origin: T) -> Self {
        Mimic { multiplier, origin }
    }
    /// Calculate the mimic joint position
    ///
    /// # Examples
    ///
    /// ```
    /// let m = k::joint::Mimic::<f64>::new(1.0, 0.5);
    /// assert_eq!(m.mimic_position(0.2), 0.7); // 0.2 * 1.0 + 0.5
    /// ```
    ///
    /// ```
    /// let m = k::joint::Mimic::<f64>::new(-2.0, -0.4);
    /// assert_eq!(m.mimic_position(0.2), -0.8); // 0.2 * -2.0 - 0.4
    /// ```
    pub fn mimic_position(&self, from_position: T) -> T {
        from_position * self.multiplier + self.origin
    }
}

/// Joint with type
#[derive(Debug, Clone)]
pub struct Joint<T: Real> {
    /// Name of this joint
    pub name: String,
    /// Type of this joint
    pub joint_type: JointType<T>,
    /// position (angle) of this joint
    position: T,
    /// velocity of this joint
    velocity: T,
    /// Limits of this joint
    pub limits: Option<Range<T>>,
    /// local origin transform of joint
    origin: Isometry3<T>,
    /// cache of world transform
    world_transform_cache: RefCell<Option<Isometry3<T>>>,
    /// cache of world velocity
    world_velocity_cache: RefCell<Option<Velocity<T>>>,
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
    /// assert!(fixed.joint_position().is_none());
    ///
    /// // create rotational joint with Y-axis
    /// let rot = k::Joint::<f64>::new("r0", k::JointType::Rotational { axis: na::Vector3::y_axis() });
    /// assert_eq!(rot.joint_position().unwrap(), 0.0);
    /// ```
    ///
    pub fn new(name: &str, joint_type: JointType<T>) -> Joint<T> {
        Joint {
            name: name.to_string(),
            joint_type,
            position: T::zero(),
            velocity: T::zero(),
            limits: None,
            origin: Isometry3::identity(),
            world_transform_cache: RefCell::new(None),
            world_velocity_cache: RefCell::new(None),
        }
    }
    /// Set the position of the joint
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
    /// // Set position to fixed joint always fails
    /// assert!(fixed.set_joint_position(1.0).is_err());
    ///
    /// // Create rotational joint with Y-axis
    /// let mut rot = k::Joint::<f64>::new("r0", k::JointType::Rotational { axis: na::Vector3::y_axis() });
    /// // As default, it has not limit
    ///
    /// // Initial position is 0.0
    /// assert_eq!(rot.joint_position().unwrap(), 0.0);
    /// // If it has no limits, set_joint_position always succeeds.
    /// rot.set_joint_position(0.2).unwrap();
    /// assert_eq!(rot.joint_position().unwrap(), 0.2);
    /// ```
    ///
    pub fn set_joint_position(&mut self, position: T) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimitError {
                joint_name: self.name.to_string(),
                message: "Joint is Fixed".to_owned(),
            });
        }
        if let Some(ref range) = self.limits {
            if !range.is_valid(position) {
                return Err(JointError::OutOfLimitError {
                    joint_name: self.name.to_string(),
                    message: format!(
                        "Joint is out of range: input={}, range={:?}",
                        position, range
                    ),
                });
            }
        }
        self.position = position;
        // TODO: have to reset descendent `world_transform_cache`
        self.world_transform_cache.replace(None);
        self.world_velocity_cache.replace(None);
        Ok(())
    }
    pub fn set_joint_position_unchecked(&mut self, position: T) {
        self.position = position;
        // TODO: have to reset descendent `world_transform_cache`
        self.world_transform_cache.replace(None);
        self.world_velocity_cache.replace(None);
    }
    /// Returns the position (angle)
    #[inline]
    pub fn joint_position(&self) -> Option<T> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.position),
        }
    }

    #[inline]
    pub fn origin(&self) -> &Isometry3<T> {
        &self.origin
    }

    #[inline]
    pub fn set_origin(&mut self, origin: Isometry3<T>) {
        self.origin = origin;
        self.world_transform_cache.replace(None);
    }

    pub fn set_joint_velocity(&mut self, velocity: T) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimitError {
                joint_name: self.name.to_string(),
                message: "Joint is Fixed".to_owned(),
            });
        }
        self.velocity = velocity;
        self.world_velocity_cache.replace(None);
        Ok(())
    }

    /// Returns the velocity
    #[inline]
    pub fn joint_velocity(&self) -> Option<T> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.velocity),
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
    /// assert_eq!(lin.local_transform().translation.vector.x, 0.0);
    /// lin.set_joint_position(-1.0).unwrap();
    /// assert_eq!(lin.local_transform().translation.vector.x, -1.0);
    /// ```
    ///
    pub fn local_transform(&self) -> Isometry3<T> {
        let joint_transform = match self.joint_type {
            JointType::Fixed => Isometry3::identity(),
            JointType::Rotational { axis } => Isometry3::from_parts(
                Translation3::new(T::zero(), T::zero(), T::zero()),
                UnitQuaternion::from_axis_angle(&axis, self.position),
            ),
            JointType::Linear { axis } => Isometry3::from_parts(
                Translation3::from(axis.into_inner() * self.position),
                UnitQuaternion::identity(),
            ),
        };
        self.origin * joint_transform
    }

    #[inline]
    pub(crate) fn set_world_transform(&self, world_transform: Isometry3<T>) {
        self.world_transform_cache.replace(Some(world_transform));
    }

    #[inline]
    pub(crate) fn set_world_velocity(&self, world_velocity: Velocity<T>) {
        self.world_velocity_cache.replace(Some(world_velocity));
    }
    /// Get the result of forward kinematics
    ///
    /// The value is updated by `Chain::update_transforms`
    #[inline]
    pub fn world_transform(&self) -> Option<Isometry3<T>> {
        *self.world_transform_cache.borrow()
    }

    #[inline]
    pub fn world_velocity(&self) -> Option<Velocity<T>> {
        *self.world_velocity_cache.borrow()
    }

    #[inline]
    pub fn is_movable(&self) -> bool {
        match self.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
}

impl<T: Real> Display for Joint<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} {}", self.name, self.joint_type)
    }
}
