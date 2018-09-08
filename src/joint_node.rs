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
use na::{Isometry3, Real};
use std::fmt::{self, Display};

use errors::*;
use joint::*;
use rctree::*;

/// Parts of `Robot`
///
/// It contains joint, link (transform), and parent/children.
pub type JointNode<T> = Node<Joint<T>>;

impl<T> JointNode<T>
where
    T: Real,
{
    /// Return the name of the joint
    ///
    /// The return value is `String`, not `&str`.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = JointNode::new(Joint::<f64>::new("link_pitch", JointType::Fixed));
    /// assert_eq!(l0.joint_name(), "link_pitch");
    /// ```
    pub fn joint_name(&self) -> String {
        self.borrow().data.name.to_owned()
    }
    /// Clone the joint limits
    pub fn joint_limits(&self) -> Option<Range<T>> {
        self.borrow().data.limits.clone()
    }
    /// Updates and returns the local transform
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = JointNode::new(JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 1.0))
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .finalize());
    /// assert_eq!(l0.transform().translation.vector.z, 1.0);
    /// l0.set_joint_angle(0.6).unwrap();
    /// assert_eq!(l0.transform().translation.vector.z, 1.6);
    pub fn transform(&self) -> Isometry3<T> {
        self.borrow().data.transform()
    }
    /// Set the offset transform of the link
    pub fn set_offset(&self, trans: Isometry3<T>) {
        self.borrow_mut().data.offset = trans;
    }
    /// Set the angle of the joint
    ///
    /// If angle is out of limit, it returns Err.
    pub fn set_joint_angle(&self, angle: T) -> Result<(), JointError> {
        self.borrow_mut().data.set_angle(angle)
    }
    /// Get the angle of the joint. If it is fixed, it returns None.
    pub fn joint_angle(&self) -> Option<T> {
        self.borrow().data.angle()
    }
    /// Copy the type of the joint
    pub fn joint_type(&self) -> JointType<T> {
        self.borrow().data.joint_type
    }
    /// Returns if it has a joint angle. similar to `is_not_fixed()`
    pub fn has_joint_angle(&self) -> bool {
        match self.borrow().data.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
    pub(crate) fn parent_world_transform(&self) -> Option<Isometry3<T>> {
        match self.borrow().parent {
            Some(ref parent) => {
                let rc_parent = parent.upgrade().unwrap().clone();
                let parent_obj = rc_parent.borrow();
                parent_obj.data.world_transform()
            }
            None => Some(Isometry3::identity()),
        }
    }
    /// Get the calculated world transform.
    /// Call `Robot::update_transforms()` before using this method.
    ///
    ///  # Examples
    ///
    /// ```
    /// use k::*;
    /// use k::prelude::*;
    ///
    /// let l0 = JointNode::new(JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 0.2))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize());
    /// let l1 = JointNode::new(JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 1.0))
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = Robot::<f64>::from_root("tree0", l0);
    /// tree.set_joint_angles(&vec![3.141592 * 0.5, 0.1]).unwrap();
    /// assert!(l1.world_transform().is_none());
    /// assert!(l1.world_transform().is_none());
    /// let _poses = tree.update_transforms();
    /// assert!((l1.world_transform().unwrap().translation.vector.x - 1.1).abs() < 0.0001);
    /// assert!((l1.world_transform().unwrap().translation.vector.z - 0.2).abs() < 0.0001);
    ///
    /// // _poses[0] is as same as l0.world_transform()
    /// // _poses[1] is as same as l1.world_transform()
    pub fn world_transform(&self) -> Option<Isometry3<T>> {
        self.borrow().data.world_transform()
    }
}

impl<T: Real> Display for JointNode<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.borrow().data.fmt(f)
    }
}

impl<T> From<Joint<T>> for JointNode<T>
where
    T: Real,
{
    fn from(link: Joint<T>) -> Self {
        Self::new(link)
    }
}
