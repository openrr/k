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
//! Load [URDF](http://wiki.ros.org/urdf) format and create `k::LinkTree`
//!
use urdf_rs;

use na::{self, Real};
use std::collections::HashMap;
use std::path::Path;

use joints::*;
use link::*;
use link_node::*;
use link_tree::*;
use rctree::*;

impl<'a, T> From<&'a urdf_rs::Mimic> for Mimic<T>
where
    T: Real,
{
    fn from(urdf_mimic: &urdf_rs::Mimic) -> Self {
        Mimic::new(
            urdf_mimic.joint.clone(),
            na::convert(urdf_mimic.multiplier),
            na::convert(urdf_mimic.offset),
        )
    }
}

/// Returns nalgebra::Unit<nalgebra::Vector3> from f64 array
fn axis_from<T>(array3: [f64; 3]) -> na::Unit<na::Vector3<T>>
where
    T: Real,
{
    na::Unit::<_>::new_normalize(na::Vector3::new(
        na::convert(array3[0]),
        na::convert(array3[1]),
        na::convert(array3[2]),
    ))
}

/// Returns nalgebra::UnitQuaternion from f64 array
fn quaternion_from<T>(array3: &[f64; 3]) -> na::UnitQuaternion<T>
where
    T: Real,
{
    na::convert(na::UnitQuaternion::from_euler_angles(
        array3[0], array3[1], array3[2],
    ))
}

/// Returns nalgebra::Translation3 from f64 array
fn translation_from<T>(array3: &[f64; 3]) -> na::Translation3<T>
where
    T: Real,
{
    na::convert(na::Translation3::new(array3[0], array3[1], array3[2]))
}

impl<'a, T> From<&'a urdf_rs::Joint> for Link<T>
where
    T: Real,
{
    fn from(joint: &urdf_rs::Joint) -> Link<T> {
        let limit = if (joint.limit.upper - joint.limit.lower) == 0.0 {
            None
        } else {
            Some(Range::new(
                na::convert(joint.limit.lower),
                na::convert(joint.limit.upper),
            ))
        };
        LinkBuilder::<T>::new()
            .joint(
                &joint.name,
                match joint.joint_type {
                    urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
                        JointType::Rotational {
                            axis: axis_from(joint.axis.xyz),
                        }
                    }
                    urdf_rs::JointType::Prismatic => JointType::Linear {
                        axis: axis_from(joint.axis.xyz),
                    },
                    _ => JointType::Fixed,
                },
                limit,
            )
            .name(&joint.child.link)
            .rotation(quaternion_from(&joint.origin.rpy))
            .translation(translation_from(&joint.origin.xyz))
            .finalize()
    }
}

fn get_root_link_name(robot: &urdf_rs::Robot) -> String {
    let mut child_joint_map = HashMap::<&str, &urdf_rs::Joint>::new();
    for j in &robot.joints {
        if let Some(old) = child_joint_map.insert(&j.child.link, j) {
            warn!("old {:?} found", old);
        }
    }
    let mut parent_link_name: &str = &robot.links[0].name;
    while let Some(joint) = child_joint_map.get(&parent_link_name) {
        parent_link_name = &joint.parent.link;
    }
    parent_link_name.to_string()
}

impl<'a, T> From<&'a urdf_rs::Robot> for LinkTree<T>
where
    T: na::Real,
{
    fn from(robot: &urdf_rs::Robot) -> Self {
        let root_name = get_root_link_name(robot);
        let mut ref_nodes = Vec::new();
        let mut child_link_name_to_node = HashMap::new();
        let mut parent_link_name_to_node = HashMap::<&String, Vec<LinkNode<T>>>::new();
        let root_node = LinkBuilder::<T>::new()
            .joint("root", JointType::Fixed, None)
            .name(&root_name)
            .finalize()
            .into();
        for j in &robot.joints {
            let node = Node::new(j.into());
            child_link_name_to_node.insert(&j.child.link, node.clone());
            if parent_link_name_to_node.get(&j.parent.link).is_some() {
                parent_link_name_to_node
                    .get_mut(&j.parent.link)
                    .unwrap()
                    .push(node.clone());
            } else {
                parent_link_name_to_node.insert(&j.parent.link, vec![node.clone()]);
            }
            ref_nodes.push(node);
        }
        let mimics = robot
            .joints
            .iter()
            .filter_map(|j| {
                if j.mimic.joint != "" {
                    debug!("mimic found for {}", j.mimic.joint);
                    Some((j.name.clone(), (&j.mimic).into()))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();

        for l in &robot.links {
            info!("link={}", l.name);
            if let Some(parent_node) = child_link_name_to_node.get(&l.name) {
                if let Some(child_nodes) = parent_link_name_to_node.get(&l.name) {
                    for child_node in child_nodes.iter() {
                        info!("set parent = {}, child = {}", parent_node, child_node);
                        child_node.set_parent(parent_node);
                    }
                }
            }
        }
        // set root as parent of root joint nodes
        let root_joint_nodes = ref_nodes.iter().filter_map(|ref_node| {
            match ref_node.borrow().parent {
                None => Some(ref_node),
                Some(_) => None,
            }
        });
        for rjn in root_joint_nodes {
            info!("set parent = {}, child = {}", root_node, rjn);
            rjn.set_parent(&root_node);
        }
        // create root node..
        let mut tree = LinkTree::from_root(&robot.name, root_node);
        // add mimics
        for (name, mimic) in mimics {
            tree.mimics.insert(name, mimic);
        }
        tree
    }
}

impl<T> From<urdf_rs::Robot> for LinkTree<T>
where
    T: na::Real,
{
    fn from(robot: urdf_rs::Robot) -> Self {
        Self::from(&robot)
    }
}

impl<T> LinkTree<T>
where
    T: na::Real,
{
    pub fn from_urdf_file<P>(path: P) -> Result<Self, urdf_rs::UrdfError>
    where
        P: AsRef<Path>,
    {
        Ok(urdf_rs::read_file(path)?.into())
    }
}

#[test]
fn test_tree() {
    let robo = urdf_rs::read_file("urdf/sample.urdf").unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 1 + 6 + 6);

    let tree = LinkTree::<f32>::from(&robo);
    assert_eq!(tree.iter().count(), 13);
}

#[test]
fn test_tree_from_file() {
    let tree = LinkTree::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    assert_eq!(tree.dof(), 12);
    let names = tree
        .iter()
        .map(|link| link.joint_name())
        .collect::<Vec<_>>();
    assert_eq!(names.len(), 13);
    println!("{}", names[0]);
    assert_eq!(names[0], "root");
    assert_eq!(names[1], "r_shoulder_yaw");
}
