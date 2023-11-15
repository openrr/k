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
//! graph structure for kinematic chain
use na::{Isometry3, RealField, Translation3, UnitQuaternion};
use nalgebra as na;
use simba::scalar::SubsetOf;
use std::fmt::{self, Display};
use std::ops::Deref;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use super::errors::*;
use super::iterator::*;
use super::joint::*;
use super::link::*;

type WeakNode<T> = Weak<Mutex<NodeImpl<T>>>;

/// Node for joint tree struct
#[derive(Debug)]
pub struct NodeImpl<T>
where
    T: RealField,
{
    pub parent: Option<WeakNode<T>>,
    pub children: Vec<Node<T>>,
    pub joint: Joint<T>,
    pub mimic_parent: Option<WeakNode<T>>,
    pub mimic_children: Vec<Node<T>>,
    pub mimic: Option<Mimic<T>>,
    pub link: Option<Link<T>>,
}

/// Parts of `Chain`
///
/// It contains joint, joint (transform), and parent/children.
#[derive(Debug)]
pub struct Node<T: RealField>(pub(crate) Arc<Mutex<NodeImpl<T>>>);

impl<T> Node<T>
where
    T: RealField + SubsetOf<f64>,
{
    pub(crate) fn from_arc(arc_mutex_node: Arc<Mutex<NodeImpl<T>>>) -> Self {
        Node(arc_mutex_node)
    }

    pub fn new(joint: Joint<T>) -> Self {
        Node::<T>(Arc::new(Mutex::new(NodeImpl {
            parent: None,
            children: Vec::new(),
            joint,
            mimic_parent: None,
            mimic_children: Vec::new(),
            mimic: None,
            link: None,
        })))
    }

    pub(crate) fn lock(&self) -> MutexGuard<'_, NodeImpl<T>> {
        self.0.lock().unwrap()
    }

    pub fn joint(&self) -> JointRefGuard<'_, T> {
        JointRefGuard { guard: self.lock() }
    }

    pub fn joint_position(&self) -> Option<T> {
        self.lock().joint.joint_position()
    }

    pub fn parent(&self) -> Option<Node<T>> {
        match self.lock().parent {
            Some(ref weak) => weak.upgrade().map(Node::from_arc),
            None => None,
        }
    }

    pub fn children(&self) -> ChildrenRefGuard<'_, T> {
        ChildrenRefGuard { guard: self.lock() }
    }

    /// iter from the end to root, it contains `nodes[id]` itself
    #[inline]
    pub fn iter_ancestors(&self) -> Ancestors<T> {
        Ancestors::new(Some(self.clone()))
    }
    /// iter to the end, it contains `nodes[id]` itself
    #[inline]
    pub fn iter_descendants(&self) -> Descendants<T> {
        Descendants::new(vec![self.clone()])
    }

    /// Set parent and child relations at same time
    pub fn set_parent(&self, parent: &Node<T>) {
        self.lock().parent = Some(Arc::downgrade(&parent.0));
        parent.0.lock().unwrap().children.push(self.clone());
    }

    /// Remove parent and child relations at same time
    pub fn remove_parent(&self, parent: &Node<T>) {
        self.lock().parent = None;
        parent.0.lock().unwrap().children.retain(|x| *x != *self);
    }

    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = k::NodeBuilder::<f32>::new().into_node();
    /// let l1 = k::NodeBuilder::new().into_node();
    /// l1.set_parent(&l0);
    /// assert!(l0.is_root());
    /// assert!(!l1.is_root());
    /// ```
    pub fn is_root(&self) -> bool {
        self.lock().parent.is_none()
    }

    /// # Examples
    ///
    /// ```
    /// let l0 = k::NodeBuilder::<f64>::new().into_node();
    /// let l1 = k::NodeBuilder::new().into_node();
    /// l1.set_parent(&l0);
    /// assert!(!l0.is_end());
    /// assert!(l1.is_end());
    /// ```
    pub fn is_end(&self) -> bool {
        self.0.lock().unwrap().children.is_empty()
    }

    /// Set the origin transform of the joint
    #[inline]
    pub fn set_origin(&self, trans: Isometry3<T>) {
        self.lock().joint.set_origin(trans);
    }

    /// Get the origin transform of the joint
    #[inline]
    pub fn origin(&self) -> Isometry3<T> {
        self.joint().origin().clone()
    }

    /// Set the position (angle) of the joint
    ///
    /// If position is out of limit, it returns Err.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = NodeBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// assert!(l0.set_joint_position(1.0).is_ok());
    /// assert!(l0.set_joint_position(-1.0).is_err());
    /// ```
    ///
    /// Setting position for Fixed joint is error.
    ///
    /// ```
    /// use k::*;
    /// let l0 = NodeBuilder::new()
    ///     .joint_type(JointType::Fixed)
    ///     .into_node();
    /// assert!(l0.set_joint_position(0.0).is_err());
    /// ```
    ///
    /// `k::joint::Mimic` can be used to copy other joint's position.
    ///
    /// ```
    /// use k::*;
    /// let j0 = NodeBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// let j1 = NodeBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((0.0..=2.0).into()))
    ///     .into_node();
    /// j1.set_mimic_parent(&j0, k::joint::Mimic::new(1.5, 0.1));
    /// assert_eq!(j0.joint_position().unwrap(), 0.0);
    /// assert_eq!(j1.joint_position().unwrap(), 0.0);
    /// assert!(j0.set_joint_position(1.0).is_ok());
    /// assert_eq!(j0.joint_position().unwrap(), 1.0);
    /// assert_eq!(j1.joint_position().unwrap(), 1.6);
    /// ```
    pub fn set_joint_position(&self, position: T) -> Result<(), Error> {
        let mut node = self.lock();
        if node.mimic_parent.is_some() {
            return Ok(());
        }
        node.joint.set_joint_position(position.clone())?;
        for child in &node.mimic_children {
            let mut child_node = child.lock();
            let mimic = child_node.mimic.clone();
            match mimic {
                Some(m) => child_node
                    .joint
                    .set_joint_position(m.mimic_position(position.clone()))?,
                None => {
                    let from = self.joint().name.to_owned();
                    let to = child.joint().name.to_owned();
                    return Err(Error::MimicError { from, to });
                }
            };
        }
        Ok(())
    }

    /// Set the clamped position (angle) of the joint
    ///
    /// It refers to the joint limit and clamps the argument. This function does nothing if this is fixed joint.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = NodeBuilder::new()
    ///     .joint_type(JointType::Linear{axis: Vector3::z_axis()})
    ///     .limits(Some((-1.0..=1.0).into()))
    ///     .into_node();
    /// l0.set_joint_position_clamped(2.0);
    /// assert_eq!(l0.joint().joint_position(), Some(1.0));
    /// l0.set_joint_position_clamped(-2.0);
    /// assert_eq!(l0.joint().joint_position(), Some(-1.0));
    /// ```
    pub fn set_joint_position_clamped(&self, position: T) {
        self.0
            .lock()
            .unwrap()
            .joint
            .set_joint_position_clamped(position);
    }

    #[inline]
    pub fn set_joint_position_unchecked(&self, position: T) {
        self.0
            .lock()
            .unwrap()
            .joint
            .set_joint_position_unchecked(position);
    }

    pub(crate) fn parent_world_transform(&self) -> Option<Isometry3<T>> {
        //match self.0.borrow().parent {
        match self.parent() {
            Some(ref parent) => parent.world_transform(),
            None => Some(Isometry3::identity()),
        }
    }

    pub(crate) fn parent_world_velocity(&self) -> Option<Velocity<T>> {
        match self.parent() {
            Some(ref parent) => parent.world_velocity(),
            None => Some(Velocity::zero()),
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
    /// let l0 = NodeBuilder::new()
    ///     .translation(Translation3::new(0.0, 0.0, 0.2))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .into_node();
    /// let l1 = NodeBuilder::new()
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
        self.joint().world_transform()
    }
    #[inline]
    pub fn world_velocity(&self) -> Option<Velocity<T>> {
        self.joint().world_velocity()
    }

    pub fn mimic_parent(&self) -> Option<Node<T>> {
        match self.lock().mimic_parent {
            Some(ref weak) => weak.upgrade().map(Node::from_arc),
            None => None,
        }
    }

    pub fn set_mimic_parent(&self, parent: &Node<T>, mimic: Mimic<T>) {
        self.lock().mimic_parent = Some(Arc::downgrade(&parent.0));
        parent.lock().mimic_children.push(self.clone());
        self.lock().mimic = Some(mimic);
    }

    pub fn set_link(&self, link: Option<Link<T>>) {
        self.lock().link = link;
    }

    pub fn link(&self) -> OptionLinkRefGuard<'_, T> {
        OptionLinkRefGuard { guard: self.lock() }
    }
}

impl<T> ::std::clone::Clone for Node<T>
where
    T: RealField,
{
    fn clone(&self) -> Self {
        Node::<T>(self.0.clone())
    }
}

impl<T> PartialEq for Node<T>
where
    T: RealField,
{
    fn eq(&self, other: &Node<T>) -> bool {
        std::ptr::eq(&*self.0, &*other.0)
    }
}

impl<T: RealField + SubsetOf<f64>> Display for Node<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let inner = self.lock();
        inner.joint.fmt(f)?;

        if let Some(l) = &inner.link {
            write!(f, " => /{}/", l.name)?;
        }
        Ok(())
    }
}

impl<T> From<Joint<T>> for Node<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn from(joint: Joint<T>) -> Self {
        Self::new(joint)
    }
}

macro_rules! def_ref_guard {
    ($guard_struct:ident, $target:ty, $member:ident) => {
        #[derive(Debug)]
        pub struct $guard_struct<'a, T>
        where
            T: RealField,
        {
            guard: MutexGuard<'a, NodeImpl<T>>,
        }

        impl<'a, T> Deref for $guard_struct<'a, T>
        where
            T: RealField,
        {
            type Target = $target;
            fn deref(&self) -> &Self::Target {
                &self.guard.$member
            }
        }
    };
}

/*

macro_rules! def_ref_guard_mut {
    ($guard_struct:ident, $target:ty, $member:ident) => {
        pub struct $guard_struct<'a, T>
        where
            T: RealField,
        {
            guard: RefMut<'a, NodeImpl<T>>,
        }

        impl<'a, T> Deref for $guard_struct<'a, T>
        where
            T: RealField,
        {
            type Target = $target;
            fn deref(&self) -> &Self::Target {
                &self.guard.$member
            }
        }

        impl<'a, T> DerefMut for $guard_struct<'a, T>
        where
            T: RealField,
        {
            fn deref_mut(&mut self) -> &mut $target {
                &mut self.guard.$member
            }
        }
    };
}
*/

def_ref_guard!(JointRefGuard, Joint<T>, joint);
def_ref_guard!(OptionLinkRefGuard, Option<Link<T>>, link);
//def_ref_guard!(LinkRefGuard, Link<T>, link);
def_ref_guard!(ChildrenRefGuard, Vec<Node<T>>, children);

#[derive(Debug)]
pub struct LinkRefGuard<'a, T>
where
    T: RealField,
{
    pub(crate) guard: MutexGuard<'a, NodeImpl<T>>,
}

impl<'a, T> Deref for LinkRefGuard<'a, T>
where
    T: RealField,
{
    type Target = Link<T>;
    fn deref(&self) -> &Self::Target {
        // danger
        self.guard.link.as_ref().unwrap()
    }
}

//def_ref_guard!(ParentRefGuard, Option<WeakNode<T>>, parent);

/*
pub struct ParentRefGuard<'a, T>
where
    T: RealField,
{
    guard: Ref<'a, NodeImpl<T>>,
    parent: Option<Node<T>>,
}

impl<'a, T>  ParentRefGuard<'a, T> where T:RealField {
    pub fn new(guard: Ref<'a, NodeImpl<T>>) -> Self {
        let parent = guard.parent.and_then(|weak| weak.upgrade().map(|arc| Node::from_arc(arc)));
        Self {
            guard,
            parent,
        }
    }
}
*/
/*
impl<'a, T> Deref for ParentRefGuard<'a, T>
where
    T: RealField,
{
    type Target = Option<Node<T>>;
    fn deref(&self) -> &Self::Target {
        &self.parent
    }
}
*/

//def_ref_guard_mut!(JointRefGuardMut, Joint<T>, joint);

/// Build a `Link<T>`
///
/// # Examples
///
/// ```
/// use k::*;
/// let l0 = NodeBuilder::new()
///     .name("link_pitch")
///     .translation(Translation3::new(0.0, 0.1, 0.0))
///     .joint_type( JointType::Rotational{axis: Vector3::y_axis()})
///     .finalize();
/// println!("{l0:?}");
/// ```
#[derive(Debug, Clone)]
pub struct NodeBuilder<T: RealField> {
    name: String,
    joint_type: JointType<T>,
    limits: Option<Range<T>>,
    origin: Isometry3<T>,
}

impl<T> Default for NodeBuilder<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> NodeBuilder<T>
where
    T: RealField + SubsetOf<f64>,
{
    pub fn new() -> NodeBuilder<T> {
        NodeBuilder {
            name: "".to_string(),
            joint_type: JointType::Fixed,
            limits: None,
            origin: Isometry3::identity(),
        }
    }
    /// Set the name of the `Link`
    pub fn name(mut self, name: &str) -> NodeBuilder<T> {
        self.name = name.to_string();
        self
    }
    /// Set the joint which is connected to this link
    pub fn joint_type(mut self, joint_type: JointType<T>) -> NodeBuilder<T> {
        self.joint_type = joint_type;
        self
    }
    /// Set joint limits
    pub fn limits(mut self, limits: Option<Range<T>>) -> NodeBuilder<T> {
        self.limits = limits;
        self
    }
    /// Set the origin transform of this joint
    pub fn origin(mut self, origin: Isometry3<T>) -> NodeBuilder<T> {
        self.origin = origin;
        self
    }
    /// Set the translation of the origin transform of this joint
    pub fn translation(mut self, translation: Translation3<T>) -> NodeBuilder<T> {
        self.origin.translation = translation;
        self
    }
    /// Set the rotation of the origin transform of this joint
    pub fn rotation(mut self, rotation: UnitQuaternion<T>) -> NodeBuilder<T> {
        self.origin.rotation = rotation;
        self
    }
    /// Create `Joint` instance
    pub fn finalize(self) -> Joint<T> {
        let mut joint = Joint::new(&self.name, self.joint_type);
        joint.set_origin(self.origin);
        joint.limits = self.limits;
        joint
    }
    /// Create `Node` instead of `Joint` as output
    pub fn into_node(self) -> Node<T> {
        self.finalize().into()
    }
}

/// set parents easily
///
/// ```
/// use k::connect;
/// # fn main() {
/// let l0 = k::NodeBuilder::<f64>::new().into_node();
/// let l1 = k::NodeBuilder::new().into_node();
/// let l2 = k::NodeBuilder::new().into_node();
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
        $crate::connect!($y => $($rest)*);
    };
}
