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
use super::joint_type::*;
use super::range::*;
use super::velocity::*;
use crate::errors::*;
use na::{Isometry3, RealField, Translation3, UnitQuaternion};
use nalgebra as na;
use simba::scalar::SubsetOf;
use std::cell::RefCell;
use std::fmt::{self, Display};

/// Joint with type
#[derive(Debug, Clone)]
pub struct Joint<T: RealField> {
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
    T: RealField + SubsetOf<f64>,
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
    pub fn set_joint_position(&mut self, position: T) -> Result<(), Error> {
        if !self.is_movable() {
            return Err(Error::SetToFixedError {
                joint_name: self.name.to_string(),
            });
        }
        if let Some(ref range) = self.limits {
            if !range.is_valid(position) {
                return Err(Error::OutOfLimitError {
                    joint_name: self.name.to_string(),
                    position: na::convert(position),
                    max_limit: na::convert(range.max),
                    min_limit: na::convert(range.min),
                });
            }
        }
        self.position = position;
        // TODO: have to reset descendent `world_transform_cache`
        self.world_transform_cache.replace(None);
        self.world_velocity_cache.replace(None);
        Ok(())
    }
    /// Set the clamped position of the joint
    ///
    /// It refers to the joint limit and clamps the argument. This function does nothing if this is fixed joint.
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// // Create rotational joint with Y-axis
    /// let mut rot = k::Joint::<f64>::new("r0", k::JointType::Rotational { axis: na::Vector3::y_axis() });
    ///
    /// let limits = k::joint::Range::new(-1.0, 1.0);
    /// rot.limits = Some(limits);
    ///
    /// // Initial position is 0.0
    /// assert_eq!(rot.joint_position().unwrap(), 0.0);
    /// rot.set_joint_position_clamped(2.0);
    /// assert_eq!(rot.joint_position().unwrap(), 1.0);
    /// rot.set_joint_position_clamped(-2.0);
    /// assert_eq!(rot.joint_position().unwrap(), -1.0);
    /// ```
    ///
    pub fn set_joint_position_clamped(&mut self, position: T) {
        if !self.is_movable() {
            return;
        }
        let position_clamped = if let Some(ref range) = self.limits {
            range.clamp(position)
        } else {
            position
        };
        self.set_joint_position_unchecked(position_clamped);
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

    pub fn set_joint_velocity(&mut self, velocity: T) -> Result<(), Error> {
        if let JointType::Fixed = self.joint_type {
            return Err(Error::SetToFixedError {
                joint_name: self.name.to_string(),
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

impl<T: RealField> Display for Joint<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} {}", self.name, self.joint_type)
    }
}
