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
//! Load [URDF](http://wiki.ros.org/urdf) format and create `k::Chain`
//!
use urdf_rs;

use na::{self, Isometry3, Real};
use std::collections::HashMap;
use std::path::Path;

use chain::*;
use joint::*;
use joint_node::*;

pub const ROOT_JOINT_NAME: &str = "root";

pub fn isometry_from<T: Real>(origin_element: &urdf_rs::Pose) -> Isometry3<T> {
    Isometry3::from_parts(
        translation_from(&origin_element.xyz),
        quaternion_from(&origin_element.rpy),
    )
}

impl<'a, T> From<&'a urdf_rs::Mimic> for Mimic<T>
where
    T: Real,
{
    fn from(urdf_mimic: &urdf_rs::Mimic) -> Self {
        Mimic::new(
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
pub fn quaternion_from<T>(array3: &[f64; 3]) -> na::UnitQuaternion<T>
where
    T: Real,
{
    na::convert(na::UnitQuaternion::from_euler_angles(
        array3[0], array3[1], array3[2],
    ))
}

/// Returns nalgebra::Translation3 from f64 array
pub fn translation_from<T>(array3: &[f64; 3]) -> na::Translation3<T>
where
    T: Real,
{
    na::convert(na::Translation3::new(array3[0], array3[1], array3[2]))
}

impl<'a, T> From<&'a urdf_rs::Joint> for Joint<T>
where
    T: Real,
{
    fn from(joint: &urdf_rs::Joint) -> Joint<T> {
        let limit = if (joint.limit.upper - joint.limit.lower) == 0.0 {
            None
        } else {
            Some(Range::new(
                na::convert(joint.limit.lower),
                na::convert(joint.limit.upper),
            ))
        };
        JointBuilder::<T>::new()
            .name(&joint.name)
            .joint_type(match joint.joint_type {
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
                    JointType::Rotational {
                        axis: axis_from(joint.axis.xyz),
                    }
                }
                urdf_rs::JointType::Prismatic => JointType::Linear {
                    axis: axis_from(joint.axis.xyz),
                },
                _ => JointType::Fixed,
            })
            .limits(limit)
            .rotation(quaternion_from(&joint.origin.rpy))
            .translation(translation_from(&joint.origin.xyz))
            .finalize()
    }
}

impl<'a, T> From<&'a urdf_rs::Robot> for Chain<T>
where
    T: na::Real,
{
    fn from(robot: &urdf_rs::Robot) -> Self {
        let mut ref_nodes = Vec::new();
        let mut child_link_name_to_node = HashMap::new();
        let mut joint_name_to_node = HashMap::new();
        let mut parent_link_name_to_node = HashMap::<&String, Vec<JointNode<T>>>::new();
        let root_node = JointBuilder::<T>::new()
            .name(ROOT_JOINT_NAME)
            .finalize()
            .into();
        for j in &robot.joints {
            let node = JointNode::<T>::new(j.into());
            child_link_name_to_node.insert(&j.child.link, node.clone());
            if parent_link_name_to_node.get(&j.parent.link).is_some() {
                parent_link_name_to_node
                    .get_mut(&j.parent.link)
                    .unwrap()
                    .push(node.clone());
            } else {
                parent_link_name_to_node.insert(&j.parent.link, vec![node.clone()]);
            }
            ref_nodes.push(node.clone());
            joint_name_to_node.insert(j.name.clone(), node);
        }
        for l in &robot.links {
            info!("link={}", l.name);
            if let Some(mut parent_node) = child_link_name_to_node.get_mut(&l.name) {
                if let Some(child_nodes) = parent_link_name_to_node.get(&l.name) {
                    for child_node in child_nodes.iter() {
                        info!("set parent = {}, child = {}", parent_node, child_node);
                        child_node.set_parent(parent_node);
                    }
                }
            }
        }
        // add mimics
        for j in &robot.joints {
            if j.mimic.joint != "" {
                debug!("mimic found for {}", j.mimic.joint);
                let mut child = joint_name_to_node.get_mut(&j.name).unwrap().clone();
                let parent = joint_name_to_node.get(&j.mimic.joint).unwrap();
                child.set_mimic_parent(parent, (&j.mimic).into());
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
        Chain::from_root(root_node)
    }
}

impl<T> From<urdf_rs::Robot> for Chain<T>
where
    T: na::Real,
{
    fn from(robot: urdf_rs::Robot) -> Self {
        Self::from(&robot)
    }
}

impl<T> Chain<T>
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

/// Useful function to deal about 'Links' of URDF
///
/// `k` deals only `Joint`s of URDF. But links is connected
/// to joint always, it is easily find which joint is the
/// parent of the link.
///
/// # Examples
///
/// ```
/// extern crate urdf_rs;
/// extern crate k;
///
/// let urdf_robot = urdf_rs::read_file("urdf/sample.urdf").unwrap();
/// let map = k::urdf::link_to_joint_map(&urdf_robot);
/// assert_eq!(map.get("root_body").unwrap(), k::urdf::ROOT_JOINT_NAME);
/// assert_eq!(map.get("r_wrist2").unwrap(), "r_wrist_pitch");
/// assert!(map.get("no_exist_link").is_none());
/// ```
pub fn link_to_joint_map(urdf_robot: &urdf_rs::Robot) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for j in &urdf_robot.joints {
        map.insert(j.child.link.to_owned(), j.name.to_owned());
    }
    for l in &urdf_robot.links {
        if map.get(&l.name).is_none() {
            map.insert(l.name.to_owned(), ROOT_JOINT_NAME.to_owned());
        }
    }
    map
}

#[test]
fn test_tree() {
    let robo = urdf_rs::read_file("urdf/sample.urdf").unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 1 + 6 + 6);

    let tree = Chain::<f32>::from(&robo);
    assert_eq!(tree.iter().count(), 13);
}

#[test]
fn test_tree_from_file() {
    let tree = Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    assert_eq!(tree.dof(), 12);
    let names = tree.iter().map(|joint| joint.name()).collect::<Vec<_>>();
    assert_eq!(names.len(), 13);
    println!("{}", names[0]);
    assert_eq!(names[0], "root");
    assert_eq!(names[1], "r_shoulder_yaw");
}
