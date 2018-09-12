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
use std::cell::{Ref, RefCell, RefMut};
use std::fmt::{self, Display};
use std::rc::{Rc, Weak};

use errors::*;
use joint::*;
use iterator::*;

type WeakNode<T> = Weak<RefCell<NodeImpl<T>>>;

#[derive(Debug, Clone)]
/// Node for joint tree struct
pub(crate) struct NodeImpl<T>
where
    T: Real,
{
    pub parent: Option<WeakNode<T>>,
    pub children: Vec<JointNode<T>>,
    pub joint: Joint<T>,
    pub mimic_parent: Option<WeakNode<T>>,
    pub mimic_children: Vec<JointNode<T>>,
    pub mimic: Option<Mimic<T>>,
}

/// Parts of `Chain`
///
/// It contains joint, joint (transform), and parent/children.
#[derive(Debug)]
pub struct JointNode<T: Real>(pub(crate) Rc<RefCell<NodeImpl<T>>>);

impl<T> JointNode<T>
where
    T: Real,
{
    pub fn new(joint: Joint<T>) -> Self {
        JointNode::<T>(Rc::new(RefCell::new(NodeImpl {
            parent: None,
            children: Vec::new(),
            joint: joint,
            mimic_parent: None,
            mimic_children: Vec::new(),
            mimic: None,
        })))
    }
    #[inline]
    pub(crate) fn borrow<'a>(&'a self) -> Ref<'a, NodeImpl<T>> {
        self.0.borrow()
    }
    #[inline]
    pub(crate) fn borrow_mut<'a>(&'a self) -> RefMut<'a, NodeImpl<T>> {
        self.0.borrow_mut()
    }

    /// iter from the end to root, it contains nodes[id] itself
    #[inline]
    pub fn iter_ancestors(&self) -> Ancestors<T> {
        Ancestors::new(Some(self.clone()))
    }
    /// iter to the end, it contains nodes[id] itself
    #[inline]
    pub fn iter_descendants(&self) -> Descendants<T> {
        Descendants::new(vec![self.clone()])
    }

    /// Set parent and child relations at same time
    pub fn set_parent(&self, parent: &JointNode<T>) {
        self.borrow_mut().parent = Some(Rc::downgrade(&parent.0));
        parent.borrow_mut().children.push(self.clone());
    }

    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = k::JointBuilder::<f32>::new().into_node();
    /// let l1 = k::JointBuilder::new().into_node();
    /// l1.set_parent(&l0);
    /// assert!(l0.is_root());
    /// assert!(!l1.is_root());
    /// ```
    pub fn is_root(&self) -> bool {
        self.borrow().parent.is_none()
    }

    /// # Examples
    ///
    /// ```
    /// let l0 = k::JointBuilder::<f64>::new().into_node();
    /// let l1 = k::JointBuilder::new().into_node();
    /// l1.set_parent(&l0);
    /// assert!(!l0.is_end());
    /// assert!(l1.is_end());
    /// ```
    pub fn is_end(&self) -> bool {
        self.borrow().children.is_empty()
    }
}

impl<T> ::std::clone::Clone for JointNode<T>
where
    T: Real,
{
    fn clone(&self) -> Self {
        JointNode::<T>(self.0.clone())
    }
}

impl<T> PartialEq for JointNode<T>
where
    T: Real,
{
    fn eq(&self, other: &JointNode<T>) -> bool {
        &*self.0 as *const RefCell<NodeImpl<T>> == &*other.0 as *const RefCell<NodeImpl<T>>
    }
}

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
        self.borrow().joint.name.to_owned()
    }
    /// Clone the joint limits
    #[inline]
    pub fn limits(&self) -> Option<Range<T>> {
        self.borrow().joint.limits.clone()
    }
    /// Updates and returns the local transform
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = JointBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 1.0))
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .into_node();
    /// assert_eq!(l0.transform().translation.vector.z, 1.0);
    /// l0.set_position(0.6).unwrap();
    /// assert_eq!(l0.transform().translation.vector.z, 1.6);
    /// ```
    #[inline]
    pub fn transform(&self) -> Isometry3<T> {
        self.borrow().joint.transform()
    }
    /// Set the offset transform of the joint
    #[inline]
    pub fn set_offset(&self, trans: Isometry3<T>) {
        self.borrow_mut().joint.offset = trans;
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
    /// let j0 = JointBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// let j1 = JointBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// j1.set_mimic_parent(&j0, k::Mimic::new(1.5, 0.1));
    /// assert_eq!(j0.position().unwrap(), 0.0);
    /// assert_eq!(j1.position().unwrap(), 0.0);
    /// assert!(j0.set_position(1.0).is_ok());
    /// assert_eq!(j0.position().unwrap(), 1.0);
    /// assert_eq!(j1.position().unwrap(), 1.6);
    /// ```
    pub fn set_position(&self, position: T) -> Result<(), JointError> {
        let mut node = self.borrow_mut();
        if node.mimic_parent.is_some() {
            return Ok(());
        }
        node.joint.set_position(position)?;
        for child in &node.mimic_children {
            let mut child_node = child.borrow_mut();
            let mimic = child_node.mimic.clone();
            match mimic {
                Some(m) => child_node.joint.set_position(m.mimic_position(position))?,
                None => {
                    return Err(JointError::MimicError {
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
        self.borrow_mut().joint.set_position_unchecked(position);
    }
    /// Get the position of the joint. If it is fixed, it returns None.
    #[inline]
    pub fn position(&self) -> Option<T> {
        self.borrow().joint.position()
    }
    /// Copy the type of the joint
    #[inline]
    pub fn joint_type(&self) -> JointType<T> {
        self.borrow().joint.joint_type
    }
    /// Returns if it has a joint position. similar to `is_not_fixed()`
    #[inline]
    pub fn is_movable(&self) -> bool {
        match self.borrow().joint.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
    pub(crate) fn parent_world_transform(&self) -> Option<Isometry3<T>> {
        match self.borrow().parent {
            Some(ref parent) => {
                let rc_parent = parent.upgrade().unwrap().clone();
                let parent_obj = rc_parent.borrow();
                parent_obj.joint.world_transform()
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
        self.borrow().joint.world_transform()
    }
    pub fn set_mimic_parent(&self, parent: &JointNode<T>, mimic: Mimic<T>) {
        self.borrow_mut().mimic_parent = Some(Rc::downgrade(&parent.0));
        parent.borrow_mut().mimic_children.push(self.clone());
        self.borrow_mut().mimic = Some(mimic);
    }
}

impl<T: Real> Display for JointNode<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.borrow().joint.fmt(f)
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
    /// Create `Joint` instance
    pub fn finalize(self) -> Joint<T> {
        let mut joint = Joint::new(&self.name, self.joint_type);
        joint.limits = self.limits;
        joint.offset = self.offset;
        joint
    }
    /// Create `JointNode` instead of `Joint` as output
    pub fn into_node(self) -> JointNode<T> {
        self.finalize().into()
    }
}

/// set parents easily
/// 
/// ```
/// #[macro_use] extern crate k;
/// # fn main() {
/// let l0 = k::JointBuilder::<f64>::new().into_node();
/// let l1 = k::JointBuilder::new().into_node();
/// let l2 = k::JointBuilder::new().into_node();
/// 
/// // This is the same as below
/// // l1.set_parent(&l0);
/// // l2.set_parent(&l1);
/// connect![l0 => l1 => l2];
/// 
/// assert!(l0.is_root());
/// assert!(!l1.is_root());
/// assert!(!l1.is_end());
/// assert!(l2.is_end());
/// # }
/// ```
#[macro_export]
macro_rules! connect {
    ($x:expr => $y:expr) => {
        $y.set_parent(&$x);
    };
    ($x:expr => $y:expr => $($rest:tt)+) => {
        $y.set_parent(&$x);
        connect!($y => $($rest)*);
    };
}
