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
use std::collections::HashMap;
use std::fmt::{self, Display};

use errors::*;
use joints::*;
use links::*;
use rctree::*;
use traits::*;

/// Parts of `LinkTree`
///
/// It contains joint, link (transform), and parent/children.
pub type LinkNode<T> = Node<Link<T>>;

impl<T> LinkNode<T>
where
    T: Real,
{
    /// Return the name of the link
    ///
    /// The return value is `String`, not `&str`.
    /// If you want to check the link name, and don't want to store it,
    /// try to use `is_link_name()` instead.
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// assert_eq!(l0.link_name(), "link0");
    /// ```
    pub fn link_name(&self) -> String {
        self.borrow().data.name.to_owned()
    }
    /// Return the name of the joint
    ///
    /// The return value is `String`, not `&str`.
    /// If you want to check the joint name, and don't want to store it,
    /// try to use `is_joint_name()` instead.
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// assert_eq!(l0.joint_name(), "link_pitch");
    /// ```
    pub fn joint_name(&self) -> String {
        self.borrow().data.joint_name().to_owned()
    }
    pub fn is_joint_name(&self, name: &str) -> bool {
        self.borrow().data.joint_name() == name
    }
    pub fn is_link_name(&self, name: &str) -> bool {
        self.borrow().data.name == name
    }
    pub fn joint_limits(&self) -> Option<Range<T>> {
        self.borrow().data.joint.limits.clone()
    }
    /// Updates and returns the transform of the end of the joint
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
        self.borrow_mut().data.set_joint_angle(angle)
    }
    /// Get the angle of the joint. If it is fixed, it returns None.
    pub fn joint_angle(&self) -> Option<T> {
        self.borrow().data.joint.angle()
    }
    /// Copy the type of the joint
    pub fn joint_type(&self) -> JointType<T> {
        self.borrow().data.joint.joint_type
    }
    /// Returns if it has a joint angle. similar to `is_not_fixed()`
    pub fn has_joint_angle(&self) -> bool {
        self.borrow().data.has_joint_angle()
    }
    pub(self) fn parent_world_transform(&self) -> Option<Isometry3<T>> {
        match self.borrow().parent {
            Some(ref parent) => {
                let rc_parent = parent.upgrade().unwrap().clone();
                let parent_obj = rc_parent.borrow();
                let cache = parent_obj.data.world_transform();
                match *cache {
                    Some(trans) => Some(trans),
                    None => None,
                }
            }
            None => Some(Isometry3::identity()),
        }
    }
}

impl<T: Real> Display for LinkNode<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.borrow().data.fmt(f)
    }
}

/*
/// Kinematic chain using `Rc<RefCell<LinkNode<T>>>`
#[derive(Debug)]
pub struct Manipulator<T: Real> {
    pub name: String,
    pub links: Vec<LinkNode<T>>,
    pub mimics: HashMap<String, Mimic<T>>,
    links_with_joint_angle: Vec<LinkNode<T>>,
    end_link_name: Option<String>,
}

impl<T> Manipulator<T>
where
    T: Real,
{
    pub fn set_end_link_name(&mut self, name: &str) -> Result<(), String> {
        if self
            .links
            .iter()
            .find(|&ljn| ljn.is_link_name(name))
            .is_none()
        {
            Err(format!("{} not found", name).to_owned())
        } else {
            self.end_link_name = Some(name.to_owned());
            Ok(())
        }
    }
    pub fn end_link_name<'a>(&'a self) -> &'a Option<String> {
        &self.end_link_name
    }
    pub fn new(name: &str, end: &LinkNode<T>) -> Self {
        let mut links = end
            .iter_ancestors()
            .map(|ljn| ljn.clone())
            .collect::<Vec<_>>();
        links.reverse();
        let links_with_joint_angle = links
            .iter()
            .filter(|link| link.has_joint_angle())
            .map(|link| link.clone())
            .collect::<Vec<_>>();
        Manipulator {
            name: name.to_string(),
            links,
            links_with_joint_angle,
            end_link_name: None,
            mimics: HashMap::new(),
        }
    }
    pub fn from_link_tree(end_link_name: &str, tree: &LinkTree<T>) -> Option<Self> {
        tree.iter()
            .find(|&link| link.is_link_name(end_link_name))
            .map(|ljn| {
                let mut chain = Manipulator::new(end_link_name, ljn);
                let joint_names = chain.joint_names();
                for (from, mimic) in &tree.mimics {
                    if joint_names.contains(from) {
                        chain.mimics.insert(from.to_owned(), mimic.clone());
                    }
                }
                chain
            })
    }
}

impl<T> EndTransform<T> for Manipulator<T>
where
    T: Real,
{
    fn end_transform(&self) -> Isometry3<T> {
        let mut end_transform = Isometry3::identity();
        for link in &self.links {
            end_transform *= link.transform();
            if let Some(ref end_name) = self.end_link_name {
                if link.is_link_name(end_name) {
                    return end_transform;
                }
            }
        }
        end_transform
    }
}

impl<T> HasLinks<T> for Manipulator<T>
where
    T: Real,
{
    fn link_transforms(&self) -> Vec<Isometry3<T>> {
        self.links
            .iter()
            .scan(Isometry3::identity(), |base, ljn| {
                *base *= ljn.transform();
                Some(*base)
            })
            .collect()
    }
    fn link_names(&self) -> Vec<String> {
        self.links.iter().map(|ljn| ljn.link_name()).collect()
    }
}

impl<T: Real> Display for Manipulator<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for l in &self.links {
            write!(f, "=> {}", l)?;
        }
        Ok(())
    }
}

impl<T> HasJoints<T> for Manipulator<T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError> {
        if self.links_with_joint_angle.len() != angles.len() {
            debug!("size mismatch input angles={:?}", angles);
            return Err(JointError::SizeMisMatch {
                input: angles.len(),
                required: self.links_with_joint_angle.len(),
            });
        }
        for (i, link) in self.links_with_joint_angle.iter_mut().enumerate() {
            link.set_joint_angle(angles[i])?;
        }
        for (to, mimic) in &self.mimics {
            let from_angle = self
                .links
                .iter()
                .find(|link| link.is_joint_name(&mimic.name))
                .and_then(|link| link.joint_angle())
                .ok_or_else(|| JointError::Mimic {
                    from: mimic.name.clone(),
                    to: to.to_owned(),
                })?;
            self.links
                .iter_mut()
                .find(|link| link.is_joint_name(to))
                .map(|link| link.set_joint_angle(mimic.mimic_angle(from_angle)));
        }
        Ok(())
    }
    fn joint_angles(&self) -> Vec<T> {
        self.links_with_joint_angle
            .iter()
            .map(|link| {
                link.joint_angle()
                    .expect("links_with_joint_angle must have joitn angle")
            })
            .collect()
    }
    fn joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.links_with_joint_angle
            .iter()
            .map(|link| link.joint_limits())
            .collect()
    }
    /// skip fixed joint
    fn joint_names(&self) -> Vec<String> {
        self.links_with_joint_angle
            .iter()
            .map(|link| link.joint_name())
            .collect()
    }
}
*/

/// Kinematic Tree using `LinkNode`
///
/// # Examples
///
/// ```
/// extern crate nalgebra as na;
/// extern crate k;
/// use k::prelude::*;
///
/// let l0 = k::LinkNode::new(k::LinkBuilder::new()
///     .name("link0")
///     .translation(na::Translation3::new(0.0, 0.0, 0.1))
///     .joint("link_pitch0", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
///     .finalize());
/// let l1 = k::LinkNode::new(k::LinkBuilder::new()
///     .name("link1")
///     .translation(na::Translation3::new(0.0, 0.0, 0.5))
///     .joint("link_pitch1", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
///     .finalize());
/// let l2 = k::LinkNode::new(k::LinkBuilder::new()
///     .name("hand")
///     .translation(na::Translation3::new(0.0, 0.0, 0.5))
///     .joint("fixed", k::JointType::Fixed, None)
///     .finalize());
///
/// // Sequencial joints structure
/// l1.set_parent(&l0);
/// l2.set_parent(&l1);
///
/// let mut tree = k::LinkTree::from_root("tree0", l0);
/// assert_eq!(tree.dof(), 2);
///
/// // Get joint angles
/// let angles = tree.joint_angles();
/// assert_eq!(angles.len(), 2);
/// assert_eq!(angles[0], 0.0);
/// assert_eq!(angles[1], 0.0);
///
/// // Get the initial link transforms
/// let transforms = tree.link_transforms();
/// assert_eq!(transforms.len(), 3);
/// assert_eq!(transforms[0].translation.vector.z, 0.1);
/// assert_eq!(transforms[1].translation.vector.z, 0.6);
/// assert_eq!(transforms[2].translation.vector.z, 1.1);
/// for t in transforms {
///     println!("before: {}", t);
/// }
///
/// // Set joint angles
/// tree.set_joint_angles(&vec![1.0, 2.0]).unwrap();
/// let angles = tree.joint_angles();
/// assert_eq!(angles[0], 1.0);
/// assert_eq!(angles[1], 2.0);
///
/// // Get the result of forward kinematics
/// let transforms = tree.link_transforms();
/// assert_eq!(transforms.len(), 3);
/// for t in transforms {
///     println!("before: {}", t);
/// }
/// ```
#[derive(Debug)]
pub struct LinkTree<T: Real> {
    /// Name of this `LinkTree`
    pub name: String,
    /// Information about mimic joints
    pub mimics: HashMap<String, Mimic<T>>,
    expanded_links: Vec<LinkNode<T>>,
}

fn fmt_with_indent_level<T: Real>(
    node: &LinkNode<T>,
    level: usize,
    f: &mut fmt::Formatter,
) -> fmt::Result {
    for _i in 0..level {
        write!(f, "    ")?;
    }
    write!(f, "{}\n", node)?;
    for c in &node.borrow().children {
        fmt_with_indent_level(c, level + 1, f)?
    }
    Ok(())
}

impl<T: Real> Display for LinkTree<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt_with_indent_level(&self.iter().next().unwrap(), 0, f)
    }
}

impl<T: Real> LinkTree<T> {
    /// Create LinkTree from root link
    ///
    /// # Arguments
    ///
    /// * `root_link` - root node of the links.
    ///
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// let l1 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = k::LinkTree::from_root("tree0", l0);
    /// ```
    pub fn from_root(name: &str, root_link: LinkNode<T>) -> Self {
        let expanded_links = root_link
            .iter_descendants()
            .map(|ln| ln.clone())
            .collect::<Vec<_>>();
        LinkTree {
            name: name.to_string(),
            mimics: HashMap::new(),
            expanded_links,
        }
    }
    pub fn from_end(name: &str, end_link: LinkNode<T>) -> Self {
        let mut links = end_link
            .iter_ancestors()
            .map(|ljn| ljn.clone())
            .collect::<Vec<_>>();
        links.reverse();
        LinkTree {
            name: name.to_string(),
            mimics: HashMap::new(),
            expanded_links: links,
        }
    }
    /// Iterate for all link nodes
    ///
    /// The order is from parent to children. You can assume that parent is already iterated.
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Fixed, None)
    ///     .finalize());
    /// let l1 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = k::LinkTree::from_root("tree0", l0);
    /// let names = tree.iter().map(|link| link.link_name()).collect::<Vec<_>>();
    /// assert_eq!(names.len(), 2);
    /// assert_eq!(names[0], "link0");
    /// assert_eq!(names[1], "link1");
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = &LinkNode<T>> {
        self.expanded_links.iter()
    }
    /// Calculate the degree of freedom
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Fixed, None)
    ///     .finalize());
    /// let l1 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = k::LinkTree::from_root("tree0", l0);
    /// assert_eq!(tree.dof(), 1);
    /// ```
    pub fn dof(&self) -> usize {
        self.iter().filter(|link| link.has_joint_angle()).count()
    }
}

impl<T> HasJoints<T> for LinkTree<T>
where
    T: Real,
{
    /// Get the angles of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    fn joint_angles(&self) -> Vec<T> {
        self.iter().filter_map(|link| link.joint_angle()).collect()
    }

    /// Set the angles of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    fn set_joint_angles(&mut self, angles_vec: &[T]) -> Result<(), JointError> {
        let dof = self.iter().filter(|link| link.has_joint_angle()).count();
        if angles_vec.len() != dof {
            return Err(JointError::SizeMisMatch {
                input: angles_vec.len(),
                required: dof,
            });
        }
        for (mut link, angle) in self
            .iter()
            .filter(|link| link.has_joint_angle())
            .zip(angles_vec.iter())
        {
            link.set_joint_angle(*angle)?;
        }
        for (to, mimic) in &self.mimics {
            let from_angle = self
                .iter()
                .find(|link| link.is_joint_name(&mimic.name))
                .and_then(|link| link.joint_angle())
                .ok_or(JointError::Mimic {
                    from: mimic.name.clone(),
                    to: to.to_owned(),
                })?;
            match self.iter().find(|link| link.is_joint_name(to)) {
                Some(target) => target.set_joint_angle(mimic.mimic_angle(from_angle))?,
                None => {
                    return Err(JointError::Mimic {
                        from: mimic.name.clone(),
                        to: to.to_owned(),
                    })
                }
            }
        }
        Ok(())
    }
    fn joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.iter()
            .filter(|link| link.has_joint_angle())
            .map(|link| link.joint_limits())
            .collect()
    }
    fn joint_names(&self) -> Vec<String> {
        self.iter()
            .filter(|link| link.has_joint_angle())
            .map(|link| link.joint_name())
            .collect()
    }
}

impl<T> HasLinks<T> for LinkTree<T>
where
    T: Real,
{
    fn link_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|ljn| {
                let parent_transform = ljn.parent_world_transform().expect("cache must exist");
                let trans = parent_transform * ljn.transform();
                ljn.borrow_mut().data.set_world_transform(trans);
                trans
            })
            .collect()
    }
    fn link_names(&self) -> Vec<String> {
        self.iter().map(|link| link.link_name()).collect()
    }
}

#[test]
fn it_works() {
    use na;
    let l0 = LinkBuilder::new()
        .name("link0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint(
            "j0",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l1 = LinkBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j1",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l2 = LinkBuilder::new()
        .name("link2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j2",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l3 = LinkBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint(
            "j3",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l4 = LinkBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j4",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l5 = LinkBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j5",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();

    let ljn0 = Node::new(l0);
    let ljn1 = Node::new(l1);
    let ljn2 = Node::new(l2);
    let ljn3 = Node::new(l3);
    let ljn4 = Node::new(l4);
    let ljn5 = Node::new(l5);
    ljn1.set_parent(&ljn0);
    ljn2.set_parent(&ljn1);
    ljn3.set_parent(&ljn2);
    ljn4.set_parent(&ljn0);
    ljn5.set_parent(&ljn4);

    let names = ljn0
        .iter_descendants()
        .map(|ljn| ljn.joint_name())
        .collect::<Vec<_>>();
    println!("{}", ljn0);
    assert_eq!(names.len(), 6);
    println!("names = {:?}", names);
    let angles = ljn0
        .iter_descendants()
        .map(|ljn| ljn.joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    fn get_z(ljn: &LinkNode<f32>) -> f32 {
        match ljn.parent_world_transform() {
            Some(iso) => iso.translation.vector.z,
            None => 0.0f32,
        }
    }

    let poses = ljn0
        .iter_descendants()
        .map(|ljn| get_z(&ljn))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let _ = ljn0
        .iter_ancestors()
        .map(|ljn| ljn.set_joint_angle(-0.5))
        .collect::<Vec<_>>();
    let angles = ljn0
        .iter_descendants()
        .map(|ljn| ljn.joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    let poses = ljn0
        .iter_descendants()
        .map(|ljn| get_z(&ljn))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let arm = LinkTree::from_end("chain1", ljn3);
    assert_eq!(arm.joint_angles().len(), 4);
    println!("{:?}", arm.joint_angles());
}

#[test]
fn test_mimic() {
    use na;
    let l0 = LinkBuilder::new()
        .name("link0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint(
            "j0",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l1 = LinkBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j1",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l2 = LinkBuilder::new()
        .name("link2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j2",
            JointType::Rotational {
                axis: na::Vector3::y_axis(),
            },
            None,
        )
        .finalize();

    let ljn0 = Node::new(l0);
    let ljn1 = Node::new(l1);
    let ljn2 = Node::new(l2);
    ljn1.set_parent(&ljn0);
    ljn2.set_parent(&ljn1);

    let mut arm = LinkTree::from_root("chain1", ljn0);
    arm.mimics
        .insert(ljn2.joint_name(), Mimic::new(ljn1.joint_name(), 2.0, 0.5));

    assert_eq!(arm.joint_angles().len(), 3);
    println!("{:?}", arm.joint_angles());
    let angles = vec![0.1, 0.2, 0.3];
    arm.set_joint_angles(&angles).unwrap();
    let angles = arm.joint_angles();
    assert!(angles[0] == 0.1);
    assert!(angles[1] == 0.2);
    assert!(angles[2] == 0.9);
}
