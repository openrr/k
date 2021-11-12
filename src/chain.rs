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
use super::errors::*;
use super::joint::*;
use super::node::*;
use na::{Isometry3, RealField};
use nalgebra as na;
use simba::scalar::SubsetOf;
use std::fmt::{self, Display};
use std::ops::Deref;

/// Kinematic Chain using `Node`
///
/// # Examples
///
/// ```
/// use k::*;
/// use k::prelude::*;
///
/// let l0 = NodeBuilder::new()
///     .name("joint_pitch0")
///     .translation(Translation3::new(0.0, 0.0, 0.1))
///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
///     .into_node();
/// let l1 = NodeBuilder::new()
///     .name("joint_pitch1")
///     .translation(Translation3::new(0.0, 0.0, 0.5))
///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
///     .into_node();
/// let l2 = NodeBuilder::new()
///     .name("hand")
///     .translation(Translation3::new(0.0, 0.0, 0.5))
///     .joint_type(JointType::Fixed)
///     .into_node();
///
/// // Sequential joints structure
/// connect![l0 => l1 => l2];
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
pub struct Chain<T: RealField> {
    nodes: Vec<Node<T>>,
    movable_nodes: Vec<Node<T>>,
    dof: usize,
}

impl<T: RealField + SubsetOf<f64>> Chain<T> {
    fn fmt_with_indent_level(
        &self,
        node: &Node<T>,
        level: usize,
        f: &mut fmt::Formatter<'_>,
    ) -> fmt::Result {
        if self.nodes.iter().any(|joint| joint == node) {
            writeln!(f, "{}{}", "    ".repeat(level), node)?;
        }
        for c in node.children().iter() {
            self.fmt_with_indent_level(c, level + 1, f)?
        }
        Ok(())
    }
}

impl<T: RealField + SubsetOf<f64>> Display for Chain<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.fmt_with_indent_level(self.iter().next().unwrap(), 0, f)
    }
}

impl<T: RealField + SubsetOf<f64>> Chain<T> {
    /// Create Chain from root joint
    ///
    /// # Examples
    ///
    /// ```
    /// use k;
    ///
    /// let l0 = k::NodeBuilder::new().into_node();
    /// let l1 = k::NodeBuilder::new().into_node();
    /// l1.set_parent(&l0);
    /// let tree = k::Chain::<f32>::from_root(l0);
    /// ```
    #[allow(clippy::needless_pass_by_value)]
    pub fn from_root(root_joint: Node<T>) -> Self {
        let nodes = root_joint.iter_descendants().collect::<Vec<_>>();
        Self::from_nodes(nodes)
    }

    /// Create `Chain` from end joint. It has any branches.
    ///
    /// Do not discard root joint before create Chain.
    /// If you want to get Chain, `unwrap()` this, it is safe.
    ///
    /// # Examples
    ///
    ///
    /// ```
    /// use k::*;
    ///
    /// fn create_tree_from_end() -> Chain<f64> {
    ///   let l0 = Node::new(Joint::new("fixed0", JointType::Fixed));
    ///   let l1 = Node::new(Joint::new("fixed1", JointType::Fixed));
    ///   l1.set_parent(&l0);
    ///   Chain::from_end(&l1) // ok, because root is stored in `Chain`
    /// }
    ///
    /// let tree = create_tree_from_end(); // no problem
    /// ```
    pub fn from_end(end_joint: &Node<T>) -> Chain<T> {
        let mut nodes = end_joint.iter_ancestors().collect::<Vec<_>>();
        nodes.reverse();
        Self::from_nodes(nodes)
    }

    /// Create `Chain` from nodes.
    ///
    /// This method is public, but it is for professional use.
    ///
    /// # Examples
    ///
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = Node::new(Joint::new("fixed0", JointType::Fixed));
    /// let l1 = Node::new(Joint::new("fixed1", JointType::Fixed));
    /// l1.set_parent(&l0);
    /// let chain = Chain::<f64>::from_nodes(vec![l0, l1]);
    /// ```
    pub fn from_nodes(nodes: Vec<Node<T>>) -> Chain<T> {
        let movable_nodes = nodes
            .iter()
            .filter(|joint| joint.joint().is_movable())
            .cloned()
            .collect::<Vec<_>>();
        Chain {
            dof: movable_nodes.len(),
            movable_nodes,
            nodes,
        }
    }

    /// Create `Chain` from end node and root node, without any branches.
    /// The root node is included in the chain.
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = Node::new(Joint::new("fixed0", JointType::Fixed));
    /// let l1 = Node::new(Joint::new("fixed1", JointType::Fixed));
    /// let l2 = Node::new(Joint::new("fixed2", JointType::Fixed));
    /// let l3 = Node::new(Joint::new("fixed3", JointType::Fixed));
    /// l1.set_parent(&l0);
    /// l2.set_parent(&l1);
    /// l3.set_parent(&l2);
    /// let chain = Chain::<f32>::from_end_to_root(&l2, &l1);
    ///
    /// assert!(chain.find("fixed0").is_none()); // not included
    /// assert!(chain.find("fixed1").is_some());
    /// assert!(chain.find("fixed2").is_some());
    /// assert!(chain.find("fixed3").is_none()); // not included
    /// ```
    pub fn from_end_to_root(end_joint: &Node<T>, root_joint: &Node<T>) -> Chain<T> {
        let mut nodes = Vec::new();
        for n in end_joint.iter_ancestors() {
            nodes.push(n.clone());
            if n == *root_joint {
                break;
            }
        }
        nodes.reverse();
        Self::from_nodes(nodes)
    }

    /// Set the `Chain`'s origin
    ///
    /// # Examples
    ///
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = Node::new(Joint::new("fixed0", JointType::Fixed));
    /// let l1 = Node::new(Joint::new("fixed1", JointType::Fixed));
    /// l1.set_parent(&l0);
    /// let c = Chain::<f32>::from_end(&l1);
    /// let mut o = c.origin();
    /// assert!(o.translation.vector[0].abs() < 0.000001);
    /// o.translation.vector[0] = 1.0;
    /// c.set_origin(o);
    /// assert!((o.translation.vector[0] - 1.0).abs() < 0.000001);
    /// ```
    pub fn set_origin(&self, pose: na::Isometry3<T>) {
        self.nodes[0].set_origin(pose)
    }

    /// Get the `Chain`'s origin
    pub fn origin(&self) -> na::Isometry3<T> {
        self.nodes[0].origin()
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
    /// let l0 = Node::new(Joint::new("fixed0", JointType::Fixed));
    /// let l1 = Node::new(Joint::new("fixed1", JointType::Fixed));
    /// l1.set_parent(&l0);
    /// let tree = Chain::<f64>::from_root(l0);
    /// let names = tree.iter().map(|node| node.joint().name.to_owned()).collect::<Vec<_>>();
    /// assert_eq!(names.len(), 2);
    /// assert_eq!(names[0], "fixed0");
    /// assert_eq!(names[1], "fixed1");
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = &Node<T>> {
        self.nodes.iter()
    }

    /// Iterate for movable joints
    ///
    /// Fixed joints are ignored. If you want to manipulate on Fixed,
    /// use `iter()` instead of `iter_joints()`
    pub fn iter_joints(&self) -> impl Iterator<Item = JointRefGuard<'_, T>> {
        self.movable_nodes.iter().map(|node| node.joint())
    }

    /// Iterate for links
    pub fn iter_links(&self) -> impl Iterator<Item = LinkRefGuard<'_, T>> {
        self.nodes.iter().filter_map(|node| {
            if node.link().is_some() {
                Some(LinkRefGuard { guard: node.lock() })
            } else {
                None
            }
        })
    }
    /// Calculate the degree of freedom
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    /// let l0 = NodeBuilder::new()
    ///     .joint_type(JointType::Fixed)
    ///     .finalize()
    ///     .into();
    /// let l1 : Node<f64> = NodeBuilder::new()
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize()
    ///     .into();
    /// l1.set_parent(&l0);
    /// let tree = Chain::from_root(l0);
    /// assert_eq!(tree.dof(), 1);
    /// ```
    pub fn dof(&self) -> usize {
        self.dof
    }
    /// Find the joint by name
    ///
    /// # Examples
    ///
    /// ```
    /// use k::*;
    ///
    /// let l0 = Node::new(NodeBuilder::new()
    ///     .name("fixed")
    ///     .finalize());
    /// let l1 = Node::new(NodeBuilder::new()
    ///     .name("pitch1")
    ///     .translation(Translation3::new(0.0, 0.1, 0.0))
    ///     .joint_type(JointType::Rotational{axis: Vector3::y_axis()})
    ///     .finalize());
    /// l1.set_parent(&l0);
    /// let tree = Chain::from_root(l0);
    /// let j = tree.find("pitch1").unwrap();
    /// j.set_joint_position(0.5).unwrap();
    /// assert_eq!(j.joint_position().unwrap(), 0.5);
    /// ```
    pub fn find(&self, joint_name: &str) -> Option<&Node<T>> {
        self.iter().find(|joint| joint.joint().name == joint_name)
    }
    /// Get the positions of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    pub fn joint_positions(&self) -> Vec<T> {
        self.iter_joints()
            .map(|joint| {
                joint
                    .joint_position()
                    .expect("Must be a bug: movable joint must have position")
            })
            .collect()
    }

    /// Set the positions of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    pub fn set_joint_positions(&self, positions_vec: &[T]) -> Result<(), Error> {
        if positions_vec.len() != self.dof {
            return Err(Error::SizeMismatchError {
                input: positions_vec.len(),
                required: self.dof,
            });
        }
        for (joint, position) in self.movable_nodes.iter().zip(positions_vec.iter()) {
            joint.set_joint_position(position.clone())?;
        }
        Ok(())
    }

    /// Set the clamped positions of the joints
    ///
    /// This function is safe, in contrast to `set_joint_positions_unchecked`.
    pub fn set_joint_positions_clamped(&self, positions_vec: &[T]) {
        for (joint, position) in self.movable_nodes.iter().zip(positions_vec.iter()) {
            joint.set_joint_position_clamped(position.clone());
        }
    }

    /// Fast, but without check, dangerous `set_joint_positions`
    #[inline]
    pub fn set_joint_positions_unchecked(&self, positions_vec: &[T]) {
        for (joint, position) in self.movable_nodes.iter().zip(positions_vec.iter()) {
            joint.set_joint_position_unchecked(position.clone());
        }
    }

    /// Update world_transform() of the joints
    pub fn update_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|node| {
                let parent_transform = node.parent_world_transform().expect("cache must exist");
                let trans = parent_transform * node.joint().local_transform();
                node.joint().set_world_transform(trans.clone());
                trans
            })
            .collect()
    }

    /// Update world_velocity() of the joints
    pub fn update_velocities(&self) -> Vec<Velocity<T>> {
        self.update_transforms();
        self.iter()
            .map(|node| {
                let parent_transform = node
                    .parent_world_transform()
                    .expect("transform cache must exist");
                let parent_velocity = node
                    .parent_world_velocity()
                    .expect("velocity cache must exist");
                let velocity = match &node.joint().joint_type {
                    JointType::Fixed => parent_velocity,
                    JointType::Rotational { axis } => {
                        let parent = node.parent().expect("parent must exist");
                        let parent_vel = parent.joint().origin().translation.vector.clone();
                        Velocity::from_parts(
                            parent_velocity.translation
                                + parent_velocity.rotation.cross(
                                    &(parent_transform.rotation.to_rotation_matrix() * parent_vel),
                                ),
                            parent_velocity.rotation
                                + node
                                    .world_transform()
                                    .expect("cache must exist")
                                    .rotation
                                    .to_rotation_matrix()
                                    * (axis.clone().into_inner()
                                        * node.joint().joint_velocity().unwrap()),
                        )
                    }
                    JointType::Linear { axis } => Velocity::from_parts(
                        parent_velocity.translation
                            + node
                                .world_transform()
                                .expect("cache must exist")
                                .rotation
                                .to_rotation_matrix()
                                * (axis.clone().into_inner()
                                    * node.joint().joint_velocity().unwrap()),
                        // TODO: Is this true??
                        parent_velocity.rotation,
                    ),
                };
                node.joint().set_world_velocity(velocity.clone());
                velocity
            })
            .collect()
    }

    /// Update transforms of the links
    pub fn update_link_transforms(&self) {
        self.update_transforms();
        self.iter().for_each(|node| {
            let parent_transform = node.parent_world_transform().expect("cache must exist");
            let mut node_mut = node.lock();
            if let Some(ref mut link) = node_mut.link {
                let inertial_trans = parent_transform.clone() * link.inertial.origin();
                link.inertial.set_world_transform(inertial_trans);
                for c in &mut link.collisions {
                    let c_trans = parent_transform.clone() * c.origin();
                    c.set_world_transform(c_trans);
                }
                for v in &mut link.visuals {
                    let v_trans = parent_transform.clone() * v.origin();
                    v.set_world_transform(v_trans);
                }
            }
        });
    }
}

impl<T> Clone for Chain<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn clone(&self) -> Self {
        // first node must be root
        if self.nodes.is_empty() {
            return Self {
                nodes: vec![],
                movable_nodes: vec![],
                dof: 0,
            };
        }
        assert!(self.nodes[0].is_root());
        // Clone everything
        let mut new_nodes = self
            .nodes
            .iter()
            .map(|n| {
                let node = Node::new(n.joint().clone());
                node.set_link(n.link().clone());
                node
            })
            .collect::<Vec<_>>();
        // Connect to new nodes
        for i in 0..new_nodes.len() {
            if let Some(p) = self.nodes[i].parent() {
                // get index of p
                let parent_index = self.nodes.iter().position(|x| *x == p).unwrap();
                new_nodes[i].set_parent(&new_nodes[parent_index]);
            }
            if let Some(m) = self.nodes[i].mimic_parent() {
                let parent_index = self.nodes.iter().position(|x| *x == m).unwrap();
                new_nodes[i].set_mimic_parent(
                    &new_nodes[parent_index],
                    self.nodes[i].lock().mimic.clone().unwrap(),
                );
            }
        }
        //
        // first node must be root
        assert!(new_nodes[0].is_root());
        Chain::from_root(new_nodes.remove(0))
    }
}

#[derive(Debug)]
/// Kinematic chain without any branch.
///
/// All joints are connected sequentially.
pub struct SerialChain<T: RealField> {
    inner: Chain<T>,
}

impl<T> SerialChain<T>
where
    T: RealField + SubsetOf<f64>,
{
    /// Convert Chain into SerialChain without any check
    ///
    /// If the input Chain has any branches it causes serious bugs.
    ///
    pub fn new_unchecked(inner: Chain<T>) -> Self {
        Self { inner }
    }
    /// Convert Chain into SerialChain
    ///
    /// If the input Chain has any branches it fails.
    ///
    /// # Examples
    ///
    /// ```
    /// let node = k::NodeBuilder::<f32>::new().into_node();
    /// let chain = k::Chain::from_root(node);
    /// assert!(k::SerialChain::try_new(chain).is_some());
    /// ```
    ///
    /// ```
    /// let node0 = k::NodeBuilder::<f32>::new().into_node();
    /// let node1 = k::NodeBuilder::new().into_node();
    /// let node2 = k::NodeBuilder::new().into_node();
    /// node1.set_parent(&node0);
    /// node2.set_parent(&node0);
    /// let chain = k::Chain::from_root(node0);
    /// assert!(k::SerialChain::try_new(chain).is_none());
    /// ```
    pub fn try_new(inner: Chain<T>) -> Option<Self> {
        {
            let num = inner.iter().count();
            for node in inner.iter().take(num - 1) {
                if node.children().len() != 1 {
                    return None;
                }
            }
        }
        Some(Self { inner })
    }
    /// Create SerialChain from the end `Node`
    ///
    /// # Examples
    ///
    /// ```
    /// let node = k::NodeBuilder::<f32>::new().into_node();
    /// let s_chain = k::SerialChain::from_end(&node);
    /// ```
    pub fn from_end(end_joint: &Node<T>) -> SerialChain<T> {
        SerialChain {
            inner: Chain::from_end(end_joint),
        }
    }

    /// Create SerialChain from the end `Node` and root `Node`.
    ///
    /// # Examples
    ///
    /// ```
    /// let node0 = k::NodeBuilder::<f32>::new().into_node();
    /// let node1 = k::NodeBuilder::<f32>::new().into_node();
    /// let node2 = k::NodeBuilder::<f32>::new().into_node();
    /// let node3 = k::NodeBuilder::<f32>::new().into_node();
    /// use k::connect;
    /// connect![node0 => node1 => node2 => node3];
    /// let s_chain = k::SerialChain::from_end_to_root(&node2, &node1);
    /// assert_eq!(s_chain.iter().count(), 2);
    /// ```
    pub fn from_end_to_root(end_joint: &Node<T>, root_joint: &Node<T>) -> SerialChain<T> {
        SerialChain {
            inner: Chain::from_end_to_root(end_joint, root_joint),
        }
    }

    /// Safely unwrap and returns inner `Chain` instance
    pub fn unwrap(self) -> Chain<T> {
        self.inner
    }
    /// Calculate transform of the end joint
    pub fn end_transform(&self) -> Isometry3<T> {
        self.iter().fold(Isometry3::identity(), |trans, joint| {
            trans * joint.joint().local_transform()
        })
    }
}

impl<T> Clone for SerialChain<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

impl<T> Display for SerialChain<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.inner.fmt(f)
    }
}

impl<T> Deref for SerialChain<T>
where
    T: RealField,
{
    type Target = Chain<T>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

#[test]
fn test_chain0() {
    use super::joint::*;
    use super::node::*;

    let joint0 = NodeBuilder::new()
        .name("j0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint1 = NodeBuilder::new()
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .name("j1")
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint2 = NodeBuilder::new()
        .name("j2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint3 = NodeBuilder::new()
        .name("j3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint4 = NodeBuilder::new()
        .name("j4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint5 = NodeBuilder::new()
        .name("j5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    joint1.set_parent(&joint0);
    joint2.set_parent(&joint1);
    joint3.set_parent(&joint2);
    joint4.set_parent(&joint0);
    joint5.set_parent(&joint4);

    let names = joint0
        .iter_descendants()
        .map(|joint| joint.joint().name.clone())
        .collect::<Vec<_>>();
    println!("{}", joint0);
    assert_eq!(names.len(), 6);
    println!("names = {:?}", names);
    let positions = joint0
        .iter_descendants()
        .map(|node| node.joint().joint_position())
        .collect::<Vec<_>>();
    println!("positions = {:?}", positions);

    fn get_z(joint: &Node<f32>) -> f32 {
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
        .map(|joint| joint.set_joint_position(-0.5))
        .collect::<Vec<_>>();
    let positions = joint0
        .iter_descendants()
        .map(|node| node.joint().joint_position())
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
    use super::node::*;

    let joint0 = NodeBuilder::new()
        .name("j0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint1 = NodeBuilder::new()
        .name("joint1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint2 = NodeBuilder::new()
        .name("joint2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    joint1.set_parent(&joint0);
    joint2.set_parent(&joint1);
    joint2.set_mimic_parent(&joint1, Mimic::new(2.0, 0.5));

    let arm = Chain::from_root(joint0);

    assert_eq!(arm.joint_positions().len(), 3);
    println!("{:?}", arm.joint_positions());
    let positions = vec![0.1, 0.2, 0.3];
    arm.set_joint_positions(&positions).unwrap();
    let positions = arm.joint_positions();
    assert!((positions[0] - 0.1f64).abs() < f64::EPSILON);
    assert!((positions[1] - 0.2f64).abs() < f64::EPSILON);
    assert!((positions[2] - 0.9f64).abs() < f64::EPSILON);
}
