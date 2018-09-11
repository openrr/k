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
use std::ops::Deref;

use errors::*;
use joint::*;
use joint_node::*;

/// Kinematic Chain using `JointNode`
///
/// # Examples
///
/// ```
/// use k::*;
/// use k::prelude::*;
///
/// // Create JointNode using `into()`
/// let l0 = JointBuilder::new()
///     .name("joint_pitch0")
///     .translation(Translation3::new(0.0, 0.0, 0.1))
///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
///     .finalize()
///     .into();
/// let l1 : JointNode<f64> = JointBuilder::new()
///     .name("joint_pitch1")
///     .translation(Translation3::new(0.0, 0.0, 0.5))
///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
///     .finalize()
///     .into();
/// // Create JointNode using `LikNode::new()`
/// let l2 = JointNode::new(JointBuilder::new()
///     .name("hand")
///     .translation(Translation3::new(0.0, 0.0, 0.5))
///     .joint_type(JointType::Fixed)
///     .finalize());
///
/// // Sequencial joints structure
/// l1.set_parent(&l0);
/// l2.set_parent(&l1);
///
/// let mut tree = Chain::from_root(l0);
/// assert_eq!(tree.dof(), 2);
///
/// // Get joint positions
/// let positions = tree.joint_positions();
/// assert_eq!(positions.len(), 2);
/// assert_eq!(positions[0], 0.0);
/// assert_eq!(positions[1], 0.0);
///
/// // Get the initial joint transforms
/// let transforms = tree.update_transforms();
/// assert_eq!(transforms.len(), 3);
/// assert_eq!(transforms[0].translation.vector.z, 0.1);
/// assert_eq!(transforms[1].translation.vector.z, 0.6);
/// assert_eq!(transforms[2].translation.vector.z, 1.1);
/// for t in transforms {
///     println!("before: {}", t);
/// }
///
/// // Set joint positions
/// tree.set_joint_positions(&vec![1.0, 2.0]).unwrap();
/// let positions = tree.joint_positions();
/// assert_eq!(positions[0], 1.0);
/// assert_eq!(positions[1], 2.0);
///
/// // Get the result of forward kinematics
/// let transforms = tree.update_transforms();
/// assert_eq!(transforms.len(), 3);
/// for t in transforms {
///     println!("after: {}", t);
/// }
/// ```
#[derive(Debug)]
pub struct Chain<T: Real> {
    contained_joints: Vec<JointNode<T>>,
}

impl<T: Real> Chain<T> {
    fn fmt_with_indent_level(
        &self,
        node: &JointNode<T>,
        level: usize,
        f: &mut fmt::Formatter,
    ) -> fmt::Result {
        if self
            .contained_joints
            .iter()
            .find(|joint| joint == &node)
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

impl<T: Real> Display for Chain<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.fmt_with_indent_level(&self.iter().next().unwrap(), 0, f)
    }
}

impl<T: Real> Chain<T> {
    /// Create Chain from root joint
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = JointNode::new(JointBuilder::new()
    ///     .name("joint_pitch")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize());
    /// let l1 = JointNode::new(JointBuilder::new()
    ///     .name("joint_pitch")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = Chain::from_root(l0);
    /// ```
    pub fn from_root(root_joint: JointNode<T>) -> Self {
        let contained_joints = root_joint
            .iter_descendants()
            .map(|joint| joint.clone())
            .collect::<Vec<_>>();
        Chain { contained_joints }
    }
    /// Create `SerialChain` from end joint
    ///
    /// Do not discard root joint before create Chain.
    /// If you want to get Chain, `unwrap()` this, it is safe.
    ///
    /// # Examples
    ///
    /// Bad case
    ///
    /// ```rust, should_panic
    /// use k::*;
    ///
    /// fn create_end_and_set_parent() -> JointNode<f64> {
    ///   let l0 = JointNode::new(Joint::new("fixed0", JointType::Fixed));
    ///   let l1 = JointNode::new(Joint::new("fixed1", JointType::Fixed));
    ///   l1.set_parent(&l0);
    ///   l1
    /// }
    ///
    /// let end = create_end_and_set_parent();
    /// let tree = Chain::from_end(&end); // panic here!
    /// ```
    ///
    /// Good case
    ///
    /// ```
    /// use k::*;
    ///
    /// fn create_tree_from_end() -> Chain<f64> {
    ///   let l0 = JointNode::new(Joint::new("fixed0", JointType::Fixed));
    ///   let l1 = JointNode::new(Joint::new("fixed1", JointType::Fixed));
    ///   l1.set_parent(&l0);
    ///   Chain::from_end(&l1).unwrap() // ok, because root is stored in `Chain`
    /// }
    ///
    /// let tree = create_tree_from_end(); // no problem
    /// ```
    pub fn from_end(end_joint: &JointNode<T>) -> SerialChain<T> {
        let mut joints = end_joint
            .iter_ancestors()
            .map(|joint| joint.clone())
            .collect::<Vec<_>>();
        joints.reverse();
        SerialChain::new_unchecked(Chain {
            contained_joints: joints,
        })
    }
    /// Iterate for all joint nodes
    ///
    /// The order is from parent to children. You can assume that parent is already iterated.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = JointNode::new(Joint::new("fixed0", JointType::Fixed));
    /// let l1 = JointNode::new(Joint::new("fixed1", JointType::Fixed));
    /// l1.set_parent(&l0);
    /// let tree = Chain::<f64>::from_root(l0);
    /// let names = tree.iter().map(|joint| joint.name()).collect::<Vec<_>>();
    /// assert_eq!(names.len(), 2);
    /// assert_eq!(names[0], "fixed0");
    /// assert_eq!(names[1], "fixed1");
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = &JointNode<T>> {
        self.contained_joints.iter()
    }

    /// Calculate the degree of freedom
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = JointBuilder::new()
    ///     .joint_type(JointType::Fixed)
    ///     .finalize()
    ///     .into();
    /// let l1 : JointNode<f64> = JointBuilder::new()
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize()
    ///     .into();
    /// l1.set_parent(&l0);
    /// let tree = Chain::from_root(l0);
    /// assert_eq!(tree.dof(), 1);
    /// ```
    pub fn dof(&self) -> usize {
        self.iter().filter(|joint| joint.has_position()).count()
    }
    /// Find the joint by name
    ///    
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = JointNode::new(JointBuilder::new()
    ///     .name("fixed")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint_type(JointType::Fixed)
    ///     .finalize());
    /// let l1 = JointNode::new(JointBuilder::new()
    ///     .name("pitch1")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = Chain::from_root(l0);
    /// let j = tree.find_joint("pitch1").unwrap();
    /// j.set_position(0.5).unwrap();
    /// assert_eq!(j.position().unwrap(), 0.5);
    /// ```
    pub fn find_joint(&self, joint_name: &str) -> Option<&JointNode<T>> {
        self.iter()
            .find(|joint| joint.borrow().data.name == joint_name)
    }
    /// Get the positions of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    pub fn joint_positions(&self) -> Vec<T> {
        self.iter().filter_map(|joint| joint.position()).collect()
    }

    /// Set the positions of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    pub fn set_joint_positions(&self, positions_vec: &[T]) -> Result<(), JointError> {
        let dof = self.iter().filter(|joint| joint.has_position()).count();
        if positions_vec.len() != dof {
            return Err(JointError::SizeMisMatch {
                input: positions_vec.len(),
                required: dof,
            });
        }
        for (mut joint, position) in self
            .iter()
            .filter(|joint| joint.has_position())
            .zip(positions_vec.iter())
        {
            joint.set_position(*position)?;
        }
        Ok(())
    }
    /// Fast, but without check, dangerous `set_joint_positions`
    #[inline]
    pub fn set_joint_positions_unchecked(&self, positions_vec: &[T]) {
        for (mut joint, position) in self
            .iter()
            .filter(|joint| joint.has_position())
            .zip(positions_vec.iter())
        {
            joint.set_position_unchecked(*position);
        }
    }
    pub fn joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.iter()
            .filter(|joint| joint.has_position())
            .map(|joint| joint.limits())
            .collect()
    }
    pub fn joint_names(&self) -> Vec<String> {
        self.iter()
            .filter(|joint| joint.has_position())
            .map(|joint| joint.name())
            .collect()
    }

    pub fn update_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|joint| {
                let parent_transform = joint.parent_world_transform().expect("cache must exist");
                let trans = parent_transform * joint.transform();
                joint.borrow_mut().data.set_world_transform(trans);
                trans
            })
            .collect()
    }
}

#[derive(Debug)]
pub struct SerialChain<T: Real> {
    inner: Chain<T>,
}

impl<T> SerialChain<T>
where
    T: Real,
{
    pub fn new_unchecked(inner: Chain<T>) -> Self {
        Self { inner }
    }
    pub fn try_new(inner: Chain<T>) -> Option<Self> {
        {
            let num = inner.iter().count();
            let joints = inner.iter().collect::<Vec<_>>();
            for i in 0..num - 1 {
                if joints[i].borrow().children.len() != 1 {
                    return None;
                }
            }
        }
        Some(Self { inner })
    }
    pub fn unwrap(self) -> Chain<T> {
        self.inner
    }
    pub fn end_transform(&self) -> Isometry3<T> {
        self.iter().fold(Isometry3::identity(), |trans, joint| {
            trans * joint.transform()
        })
    }
}

impl<T> Display for SerialChain<T>
where
    T: Real,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.inner.fmt(f)
    }
}

impl<T> Deref for SerialChain<T>
where
    T: Real,
{
    type Target = Chain<T>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

#[test]
fn it_works() {
    use super::joint::*;
    use super::rctree::Node;
    use na;

    let l0 = JointBuilder::new()
        .name("j0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l1 = JointBuilder::new()
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .name("j1")
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l2 = JointBuilder::new()
        .name("j2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l3 = JointBuilder::new()
        .name("j3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l4 = JointBuilder::new()
        .name("j4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l5 = JointBuilder::new()
        .name("j5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();

    let joint0 = Node::new(l0);
    let joint1 = Node::new(l1);
    let joint2 = Node::new(l2);
    let joint3 = Node::new(l3);
    let joint4 = Node::new(l4);
    let joint5 = Node::new(l5);
    joint1.set_parent(&joint0);
    joint2.set_parent(&joint1);
    joint3.set_parent(&joint2);
    joint4.set_parent(&joint0);
    joint5.set_parent(&joint4);

    let names = joint0
        .iter_descendants()
        .map(|joint| joint.name())
        .collect::<Vec<_>>();
    println!("{}", joint0);
    assert_eq!(names.len(), 6);
    println!("names = {:?}", names);
    let positions = joint0
        .iter_descendants()
        .map(|joint| joint.position())
        .collect::<Vec<_>>();
    println!("positions = {:?}", positions);

    fn get_z(joint: &JointNode<f32>) -> f32 {
        match joint.parent_world_transform() {
            Some(iso) => iso.translation.vector.z,
            None => 0.0f32,
        }
    }

    let poses = joint0
        .iter_descendants()
        .map(|joint| get_z(&joint))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let _ = joint0
        .iter_ancestors()
        .map(|joint| joint.set_position(-0.5))
        .collect::<Vec<_>>();
    let positions = joint0
        .iter_descendants()
        .map(|joint| joint.position())
        .collect::<Vec<_>>();
    println!("positions = {:?}", positions);

    let poses = joint0
        .iter_descendants()
        .map(|joint| get_z(&joint))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let arm = Chain::from_end(&joint3);
    assert_eq!(arm.joint_positions().len(), 4);
    println!("{:?}", arm.joint_positions());
}

#[test]
fn test_mimic() {
    use super::joint::*;
    use super::rctree::Node;
    use na;

    let l0 = JointBuilder::new()
        .name("j0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l1 = JointBuilder::new()
        .name("joint1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();
    let l2 = JointBuilder::new()
        .name("joint2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .finalize();

    let joint0 = Node::new(l0);
    let joint1 = Node::new(l1);
    let joint2 = Node::new(l2);
    joint1.set_parent(&joint0);
    joint2.set_parent(&joint1);
    joint2.set_mimic_parent(&joint1, Mimic::new(2.0, 0.5));

    let arm = Chain::from_root(joint0);

    assert_eq!(arm.joint_positions().len(), 3);
    println!("{:?}", arm.joint_positions());
    let positions = vec![0.1, 0.2, 0.3];
    arm.set_joint_positions(&positions).unwrap();
    let positions = arm.joint_positions();
    assert_eq!(positions[0], 0.1);
    assert_eq!(positions[1], 0.2);
    assert_eq!(positions[2], 0.9);
}
