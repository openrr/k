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
use std::fmt::{self, Display};

use errors::*;
use joint::*;
use rctree::*;

/// Parts of `Chain`
///
/// It contains joint, joint (transform), and parent/children.
pub type JointNode<T> = Node<Joint<T>, Mimic<T>>;

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
    /// let j0 = JointNode::new(Joint::<f64>::new("joint_pitch", JointType::Fixed));
    /// assert_eq!(j0.name(), "joint_pitch");
    /// ```
    #[inline]
    pub fn name(&self) -> String {
        self.borrow().data.name.to_owned()
    }
    /// Clone the joint limits
    #[inline]
    pub fn limits(&self) -> Option<Range<T>> {
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
    /// l0.set_position(0.6).unwrap();
    /// assert_eq!(l0.transform().translation.vector.z, 1.6);
    /// ```
    #[inline]
    pub fn transform(&self) -> Isometry3<T> {
        self.borrow().data.transform()
    }
    /// Set the offset transform of the joint
    #[inline]
    pub fn set_offset(&self, trans: Isometry3<T>) {
        self.borrow_mut().data.offset = trans;
    }
    /// Set the position (angle) of the joint
    ///
    /// If position is out of limit, it returns Err.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = JointBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// assert!(l0.set_position(1.0).is_ok());
    /// assert!(l0.set_position(-1.0).is_err());
    /// ```
    ///
    /// Setting position for Fixed joint is error.
    ///
    /// ```
    /// use k::*;
    /// let l0 = JointBuilder::new()
    ///     .joint_type(JointType::Fixed)
    ///     .into_node();
    /// assert!(l0.set_position(0.0).is_err());
    /// ```
    ///
    /// `Mimic` can be used to copy other joint's position.
    ///
    /// ```
    /// use k::*;
    /// let j0 = JointNode::new(JointBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .finalize());
    /// let j1 = JointNode::new(JointBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .finalize());
    /// j1.set_mimic_parent(&j0, k::Mimic::new(1.5, 0.1));
    /// assert_eq!(j0.position().unwrap(), 0.0);
    /// assert_eq!(j1.position().unwrap(), 0.0);
    /// assert!(j0.set_position(1.0).is_ok());
    /// assert_eq!(j0.position().unwrap(), 1.0);
    /// assert_eq!(j1.position().unwrap(), 1.6);
    /// ```
    pub fn set_position(&self, position: T) -> Result<(), JointError> {
        let mut node = self.borrow_mut();
        if node.sub_parent.is_some() {
            return Ok(());
        }
        node.data.set_position(position)?;
        for child in &node.sub_children {
            let mut child_node = child.borrow_mut();
            let mimic = child_node.sub_data.clone();
            match mimic {
                Some(m) => child_node.data.set_position(m.mimic_position(position))?,
                None => {
                    return Err(JointError::Mimic {
                        from: self.name(),
                        to: child.name(),
                        message: format!(
                        "set_position for {} -> {} failed. Mimic instance not found. child = {:?}",
                        self.name(),
                        child.name(),
                        child
                    ),
                    })
                }
            };
        }
        Ok(())
    }
    #[inline]
    pub fn set_position_unchecked(&self, position: T) {
        self.borrow_mut().data.set_position_unchecked(position);
    }
    /// Get the position of the joint. If it is fixed, it returns None.
    #[inline]
    pub fn position(&self) -> Option<T> {
        self.borrow().data.position()
    }
    /// Copy the type of the joint
    #[inline]
    pub fn joint_type(&self) -> JointType<T> {
        self.borrow().data.joint_type
    }
    /// Returns if it has a joint position. similar to `is_not_fixed()`
    #[inline]
    pub fn has_position(&self) -> bool {
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
    /// Call `Chain::update_transforms()` before using this method.
    ///
    ///  # Examples
    ///
    /// ```
    /// use k::*;
    /// use k::prelude::*;
    ///
    /// let l0 = JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 0.2))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .into_node();
    /// let l1 = JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 1.0))
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .into_node();
    /// l1.set_parent(&l0);
    /// let tree = Chain::<f64>::from_root(l0);
    /// tree.set_joint_positions(&vec![3.141592 * 0.5, 0.1]).unwrap();
    /// assert!(l1.world_transform().is_none());
    /// assert!(l1.world_transform().is_none());
    /// let _poses = tree.update_transforms();
    /// assert!((l1.world_transform().unwrap().translation.vector.x - 1.1).abs() < 0.0001);
    /// assert!((l1.world_transform().unwrap().translation.vector.z - 0.2).abs() < 0.0001);
    ///
    /// // _poses[0] is as same as l0.world_transform()
    /// // _poses[1] is as same as l1.world_transform()
    #[inline]
    pub fn world_transform(&self) -> Option<Isometry3<T>> {
        self.borrow().data.world_transform()
    }
    pub fn set_mimic_parent(&self, parent: &JointNode<T>, mimic: Mimic<T>) {
        self.set_sub_parent(parent);
        self.borrow_mut().sub_data = Some(mimic);
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
    fn from(joint: Joint<T>) -> Self {
        Self::new(joint)
    }
}


/// Build a `Link<T>`
///
/// # Examples
///
/// ```
/// use k::*;
/// let l0 = JointBuilder::new()
///     .name("link_pitch")
///     .translation(Translation3::new(0.0, 0.1, 0.0))
///     .joint_type( JointType::Rotational{axis: Vector3::y_axis()})
///     .finalize();
/// println!("{:?}", l0);
/// ```
#[derive(Debug, Clone)]
pub struct JointBuilder<T: Real> {
    name: String,
    joint_type: JointType<T>,
    limits: Option<Range<T>>,
    offset: Isometry3<T>,
}

impl<T> Default for JointBuilder<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> JointBuilder<T>
where
    T: Real,
{
    pub fn new() -> JointBuilder<T> {
        JointBuilder {
            name: "".to_string(),
            joint_type: JointType::Fixed,
            limits: None,
            offset: Isometry3::identity(),
        }
    }
    /// Set the name of the `Link`
    pub fn name(mut self, name: &str) -> JointBuilder<T> {
        self.name = name.to_string();
        self
    }
    /// Set the joint which is connected to this link
    pub fn joint_type(mut self, joint_type: JointType<T>) -> JointBuilder<T> {
        self.joint_type = joint_type;
        self
    }
    /// Set joint limits
    pub fn limits(mut self, limits: Option<Range<T>>) -> JointBuilder<T> {
        self.limits = limits;
        self
    }
    /// Set the offset transform of this joint
    pub fn offset(mut self, offset: Isometry3<T>) -> JointBuilder<T> {
        self.offset = offset;
        self
    }
    /// Set the translation of the offset transform of this joint
    pub fn translation(mut self, translation: Translation3<T>) -> JointBuilder<T> {
        self.offset.translation = translation;
        self
    }
    /// Set the rotation of the offset transform of this joint
    pub fn rotation(mut self, rotation: UnitQuaternion<T>) -> JointBuilder<T> {
        self.offset.rotation = rotation;
        self
    }
    /// Create `Link` instance
    pub fn finalize(self) -> Joint<T> {
        let mut joint = Joint::new(&self.name, self.joint_type);
        joint.limits = self.limits;
        joint.offset = self.offset;
        joint
    }
    pub fn into_node(self) -> JointNode<T> {
        self.finalize().into()
    }
}
