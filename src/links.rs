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
use na::{Isometry3, Real, Translation3, UnitQuaternion};
use std::cell::{Ref, RefCell};

use errors::*;
use joints::*;

/// Link contains a joint and a transform
///
#[derive(Debug, Clone)]
pub struct Link<T: Real> {
    pub name: String,
    /// joint instance
    pub joint: Joint<T>,
    /// local transfrom of joint
    pub transform: Isometry3<T>,
    /// cache of world transform
    world_transform_cache: RefCell<Option<Isometry3<T>>>,
}

impl<T> Link<T>
where
    T: Real,
{
    /// Construct a Link from name and joint instance
    ///
    /// You can use LinkBuilder<T> if you want.
    pub fn new(name: &str, joint: Joint<T>) -> Link<T> {
        Link {
            name: name.to_string(),
            joint: joint,
            transform: Isometry3::identity(),
            world_transform_cache: RefCell::new(None),
        }
    }
    /// Return the name of the joint
    pub fn joint_name(&self) -> &str {
        &self.joint.name
    }
    /// Updates and returns the transform of the end of the joint
    pub fn transform(&self) -> Isometry3<T> {
        self.transform * self.joint.transform()
    }
    /// Set the angle of the joint
    ///
    /// If angle is out of limit, it returns Err.
    pub fn set_joint_angle(&mut self, angle: T) -> Result<(), JointError> {
        self.joint.set_angle(angle)
    }
    /// Get the angle of the joint. If it is fixed, it returns None.
    pub fn joint_angle(&self) -> Option<T> {
        self.joint.angle()
    }
    /// Returns if it has a joint angle. similar to `is_not_fixed()`
    pub fn has_joint_angle(&self) -> bool {
        match self.joint.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
    pub(crate) fn set_world_transform(&self, world_transform: Isometry3<T>) {
        self.world_transform_cache.replace(Some(world_transform));
    }
    pub(crate) fn world_transform(&self) -> Ref<Option<Isometry3<T>>> {
        self.world_transform_cache.borrow()
    }
}

/// Build a `Link<T>`
///
/// # Examples
///
/// ```
/// extern crate nalgebra as na;
/// extern crate k;
/// let l0 = k::LinkBuilder::new()
///     .name("link1")
///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
///     .finalize();
/// println!("{:?}", l0);
/// ```
#[derive(Debug, Clone)]
pub struct LinkBuilder<T: Real> {
    name: String,
    joint: Joint<T>,
    transform: Isometry3<T>,
}

impl<T> Default for LinkBuilder<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> LinkBuilder<T>
where
    T: Real,
{
    pub fn new() -> LinkBuilder<T> {
        LinkBuilder {
            name: "".to_string(),
            joint: Joint::new("", JointType::Fixed),
            transform: Isometry3::identity(),
        }
    }
    /// Set the name of the `Link`
    pub fn name(mut self, name: &str) -> LinkBuilder<T> {
        self.name = name.to_string();
        self
    }
    /// Set the joint which is connected to this link
    pub fn joint(
        mut self,
        name: &str,
        joint_type: JointType<T>,
        limits: Option<Range<T>>,
    ) -> LinkBuilder<T> {
        self.joint = Joint::new(name, joint_type);
        self.joint.limits = limits;
        self
    }
    /// Set the transform of this link
    pub fn transform(mut self, transform: Isometry3<T>) -> LinkBuilder<T> {
        self.transform = transform;
        self
    }
    /// Set the translation of the transform of this link
    pub fn translation(mut self, translation: Translation3<T>) -> LinkBuilder<T> {
        self.transform.translation = translation;
        self
    }
    /// Set the rotation of the transform of this link
    pub fn rotation(mut self, rotation: UnitQuaternion<T>) -> LinkBuilder<T> {
        self.transform.rotation = rotation;
        self
    }
    /// Create `Link` instance
    pub fn finalize(self) -> Link<T> {
        Link {
            name: self.name,
            joint: self.joint,
            transform: self.transform,
            world_transform_cache: RefCell::new(None),
        }
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
