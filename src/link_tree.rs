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
use link_node::LinkNode;
use traits::*;

/// Kinematic Tree using `LinkNode`
///
/// # Examples
///
/// ```
/// extern crate nalgebra as na;
/// extern crate k;
/// use k::prelude::*;
///
/// // Create LinkNode using `into()`
/// let l0 = k::LinkBuilder::new()
///     .name("link0")
///     .translation(na::Translation3::new(0.0, 0.0, 0.1))
///     .joint("link_pitch0", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
///     .finalize()
///     .into();
/// let l1 : k::LinkNode<f64> = k::LinkBuilder::new()
///     .name("link1")
///     .translation(na::Translation3::new(0.0, 0.0, 0.5))
///     .joint("link_pitch1", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
///     .finalize()
///     .into();
/// // Create LinkNode using `LikNode::new()`
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
/// let transforms = tree.update_transforms();
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
/// let transforms = tree.update_transforms();
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
    contained_links: Vec<LinkNode<T>>,
}

impl<T: Real> LinkTree<T> {
    fn fmt_with_indent_level(
        &self,
        node: &LinkNode<T>,
        level: usize,
        f: &mut fmt::Formatter,
    ) -> fmt::Result {
        if self
            .contained_links
            .iter()
            .find(|link| link == &node)
            .is_some()
        {
            write!(f, "{}{}\n", "    ".repeat(level), node)?;
        }
        for c in &node.borrow().children {
            self.fmt_with_indent_level(c, level + 1, f)?
        }
        Ok(())
    }
}
impl<T: Real> Display for LinkTree<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.fmt_with_indent_level(&self.iter().next().unwrap(), 0, f)
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
    /// use k::*;
    ///
    /// let l0 = LinkNode::new(LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", JointType::Rotational{axis: Vector3::y_axis()}, None)
    ///     .finalize());
    /// let l1 = LinkNode::new(LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("link_pitch", JointType::Rotational{axis: Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = LinkTree::from_root("tree0", l0);
    /// ```
    pub fn from_root(name: &str, root_link: LinkNode<T>) -> Self {
        let contained_links = root_link
            .iter_descendants()
            .map(|ln| ln.clone())
            .collect::<Vec<_>>();
        LinkTree {
            name: name.to_string(),
            mimics: HashMap::new(),
            contained_links,
        }
    }
    /// Create `LinkTree` from end link
    ///
    /// Do not discard root link before create LinkTree.
    ///
    /// # Examples
    ///
    /// Bad case
    ///
    /// ```
    /// extern crate k;
    ///
    /// fn create_end_and_set_parent() -> k::LinkNode<f64> {
    ///   let l0 = k::LinkNode::new(k::Link::new("link0", k::Joint::new("fixed0", k::JointType::Fixed)));
    ///   let l1 = k::LinkNode::new(k::Link::new("link1", k::Joint::new("fixed1", k::JointType::Fixed)));
    ///   l1.set_parent(&l0);
    ///   l1
    /// }
    /// let end = create_end_and_set_parent();
    /// // k::LinkTree::from_end("tree0", end); // panic here!
    /// ```
    ///
    /// Good case
    ///
    /// ```
    /// use k::*;
    /// fn create_end_and_set_parent() -> k::LinkTree<f64> {
    ///   let l0 = LinkNode::new(Link::new("link0", Joint::new("fixed0", JointType::Fixed)));
    ///   let l1 = LinkNode::new(Link::new("link1", Joint::new("fixed1", JointType::Fixed)));
    ///   l1.set_parent(&l0);
    ///   LinkTree::from_end("tree0", l1) // ok, because root is stored in `LinkTree`
    /// }
    /// let end = create_end_and_set_parent(); // no problem
    /// ```
    pub fn from_end(name: &str, end_link: LinkNode<T>) -> Self {
        let mut links = end_link
            .iter_ancestors()
            .map(|link| link.clone())
            .collect::<Vec<_>>();
        links.reverse();
        LinkTree {
            name: name.to_string(),
            mimics: HashMap::new(),
            contained_links: links,
        }
    }
    /// Iterate for all link nodes
    ///
    /// The order is from parent to children. You can assume that parent is already iterated.
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = LinkNode::new(Link::new("link0", Joint::new("fixed0", JointType::Fixed)));
    /// let l1 = LinkNode::new(Link::new("link1", Joint::new("fixed1", JointType::Fixed)));
    /// l1.set_parent(&l0);
    /// let tree = LinkTree::<f64>::from_root("tree0", l0);
    /// let names = tree.iter().map(|link| link.link_name()).collect::<Vec<_>>();
    /// assert_eq!(names.len(), 2);
    /// assert_eq!(names[0], "link0");
    /// assert_eq!(names[1], "link1");
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = &LinkNode<T>> {
        self.contained_links.iter()
    }

    /// Calculate the degree of freedom
    ///
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
    /// Find the joint by name
    ///    
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("joint_fixed", k::JointType::Fixed, None)
    ///     .finalize());
    /// let l1 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("joint_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = k::LinkTree::from_root("tree0", l0);
    /// let j = tree.find_joint("joint_pitch").unwrap();
    /// j.set_joint_angle(0.5).unwrap();
    /// assert_eq!(j.joint_angle().unwrap(), 0.5);
    /// ```
    pub fn find_joint(&self, joint_name: &str) -> Option<&LinkNode<T>> {
        self.iter()
            .find(|link| link.borrow().data.joint_name() == joint_name)
    }
    /// Find the link by name
    ///    
    /// # Examples
    ///
    /// ```
    /// extern crate nalgebra as na;
    /// extern crate k;
    ///
    /// let l0 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link0")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("joint_fixed", k::JointType::Fixed, None)
    ///     .finalize());
    /// let l1 = k::LinkNode::new(k::LinkBuilder::new()
    ///     .name("link1")
    ///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
    ///     .joint("joint_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()}, None)
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = k::LinkTree::from_root("tree0", l0);
    /// tree.find_link("link1").unwrap().set_joint_angle(0.5).unwrap();
    /// assert_eq!(tree.find_link("link1").unwrap().joint_angle().unwrap(), 0.5);
    /// ```
    pub fn find_link(&self, link_name: &str) -> Option<&LinkNode<T>> {
        self.iter()
            .find(|link| link.borrow().data.name == link_name)
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
    fn set_joint_angles(&self, angles_vec: &[T]) -> Result<(), JointError> {
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
                .find_joint(&mimic.name)
                .and_then(|link| link.joint_angle())
                .ok_or(JointError::Mimic {
                    from: mimic.name.clone(),
                    to: to.to_owned(),
                })?;
            match self.find_joint(to) {
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
    fn update_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|link| {
                let parent_transform = link.parent_world_transform().expect("cache must exist");
                let trans = parent_transform * link.transform();
                link.borrow_mut().data.set_world_transform(trans);
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
    use super::link::*;
    use super::rctree::Node;
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

    let link0 = Node::new(l0);
    let link1 = Node::new(l1);
    let link2 = Node::new(l2);
    let link3 = Node::new(l3);
    let link4 = Node::new(l4);
    let link5 = Node::new(l5);
    link1.set_parent(&link0);
    link2.set_parent(&link1);
    link3.set_parent(&link2);
    link4.set_parent(&link0);
    link5.set_parent(&link4);

    let names = link0
        .iter_descendants()
        .map(|link| link.joint_name())
        .collect::<Vec<_>>();
    println!("{}", link0);
    assert_eq!(names.len(), 6);
    println!("names = {:?}", names);
    let angles = link0
        .iter_descendants()
        .map(|link| link.joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    fn get_z(link: &LinkNode<f32>) -> f32 {
        match link.parent_world_transform() {
            Some(iso) => iso.translation.vector.z,
            None => 0.0f32,
        }
    }

    let poses = link0
        .iter_descendants()
        .map(|link| get_z(&link))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let _ = link0
        .iter_ancestors()
        .map(|link| link.set_joint_angle(-0.5))
        .collect::<Vec<_>>();
    let angles = link0
        .iter_descendants()
        .map(|link| link.joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    let poses = link0
        .iter_descendants()
        .map(|link| get_z(&link))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let arm = LinkTree::from_end("chain1", link3);
    assert_eq!(arm.joint_angles().len(), 4);
    println!("{:?}", arm.joint_angles());
}

#[test]
fn test_mimic() {
    use super::link::*;
    use super::rctree::Node;
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

    let link0 = Node::new(l0);
    let link1 = Node::new(l1);
    let link2 = Node::new(l2);
    link1.set_parent(&link0);
    link2.set_parent(&link1);

    let mut arm = LinkTree::from_root("chain1", link0);
    arm.mimics
        .insert(link2.joint_name(), Mimic::new(link1.joint_name(), 2.0, 0.5));

    assert_eq!(arm.joint_angles().len(), 3);
    println!("{:?}", arm.joint_angles());
    let angles = vec![0.1, 0.2, 0.3];
    arm.set_joint_angles(&angles).unwrap();
    let angles = arm.joint_angles();
    assert!(angles[0] == 0.1);
    assert!(angles[1] == 0.2);
    assert!(angles[2] == 0.9);
}
