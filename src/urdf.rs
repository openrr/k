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

use na::{self, Isometry3, Matrix3, Real};
use std::collections::HashMap;
use std::path::Path;

use chain::*;
use joint::*;
use link::*;
use node::*;

pub const ROOT_JOINT_NAME: &str = "root";

impl<'a, T> From<&'a urdf_rs::Color> for Color<T>
where
    T: Real,
{
    fn from(urdf_color: &urdf_rs::Color) -> Self {
        Color {
            r: na::convert(urdf_color.rgba[0]),
            g: na::convert(urdf_color.rgba[1]),
            b: na::convert(urdf_color.rgba[2]),
            a: na::convert(urdf_color.rgba[3]),
        }
    }
}

impl<T> From<urdf_rs::Color> for Color<T>
where
    T: Real,
{
    fn from(urdf_color: urdf_rs::Color) -> Self {
        (&urdf_color).into()
    }
}

impl From<urdf_rs::Texture> for Texture {
    fn from(urdf_texture: urdf_rs::Texture) -> Self {
        Texture {
            filename: urdf_texture.filename,
        }
    }
}

impl<T> From<urdf_rs::Material> for Material<T>
where
    T: Real,
{
    fn from(urdf_material: urdf_rs::Material) -> Self {
        Material {
            name: urdf_material.name,
            color: urdf_material.color.into(),
            texture: urdf_material.texture.into(),
        }
    }
}

pub fn isometry_from<T: Real>(origin_element: &urdf_rs::Pose) -> Isometry3<T> {
    Isometry3::from_parts(
        translation_from(&origin_element.xyz),
        quaternion_from(&origin_element.rpy),
    )
}

impl<T> From<urdf_rs::Inertial> for Inertial<T>
where
    T: Real,
{
    fn from(urdf_inertial: urdf_rs::Inertial) -> Self {
        let i = urdf_inertial.inertia;
        Inertial::new(
            isometry_from(&urdf_inertial.origin),
            na::convert(urdf_inertial.mass.value),
            Matrix3::new(
                na::convert(i.ixx),
                na::convert(i.ixy),
                na::convert(i.ixz),
                na::convert(i.ixy),
                na::convert(i.iyy),
                na::convert(i.iyz),
                na::convert(i.ixz),
                na::convert(i.iyz),
                na::convert(i.izz),
            ),
        )
    }
}

impl<T> From<urdf_rs::Visual> for Visual<T>
where
    T: Real,
{
    fn from(urdf_visual: urdf_rs::Visual) -> Self {
        Visual::new(
            urdf_visual.name,
            isometry_from(&urdf_visual.origin),
            urdf_visual.geometry.into(),
            urdf_visual.material.into(),
        )
    }
}

impl<T> From<urdf_rs::Collision> for Collision<T>
where
    T: Real,
{
    fn from(urdf_collision: urdf_rs::Collision) -> Self {
        Collision::new(
            urdf_collision.name,
            isometry_from(&urdf_collision.origin),
            urdf_collision.geometry.into(),
        )
    }
}

impl<T> From<urdf_rs::Geometry> for Geometry<T>
where
    T: Real,
{
    fn from(urdf_geometry: urdf_rs::Geometry) -> Self {
        match urdf_geometry {
            urdf_rs::Geometry::Box { size } => Geometry::Box {
                depth: na::convert(size[0]),
                width: na::convert(size[1]),
                height: na::convert(size[2]),
            },
            urdf_rs::Geometry::Cylinder { radius, length } => Geometry::Cylinder {
                radius: na::convert(radius),
                length: na::convert(length),
            },
            urdf_rs::Geometry::Sphere { radius } => Geometry::Sphere {
                radius: na::convert(radius),
            },
            urdf_rs::Geometry::Mesh { filename, scale } => Geometry::Mesh {
                filename,
                scale: na::Vector3::new(
                    na::convert(scale[0]),
                    na::convert(scale[1]),
                    na::convert(scale[2]),
                ),
            },
        }
    }
}

impl<T> From<urdf_rs::Link> for Link<T>
where
    T: Real,
{
    fn from(urdf_link: urdf_rs::Link) -> Self {
        Link {
            name: urdf_link.name,
            inertial: urdf_link.inertial.into(),
            visuals: urdf_link.visual.into_iter().map(|v| v.into()).collect(),
            collisions: urdf_link.collision.into_iter().map(|v| v.into()).collect(),
        }
    }
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
        let mut parent_link_name_to_node = HashMap::<&String, Vec<Node<T>>>::new();
        let root_node = JointBuilder::<T>::new().name(ROOT_JOINT_NAME).into_node();
        for j in &robot.joints {
            let node = Node::<T>::new(j.into());
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
                parent_node.set_link(Some(l.clone().into()));
            } else {
                info!("root={}", l.name);
                root_node.set_link(Some(l.clone().into()));
            }
        }
        // add mimics
        for j in &robot.joints {
            if j.mimic.joint != "" {
                debug!("mimic found for {}", j.mimic.joint);
                let mut child = joint_name_to_node[&j.name].clone();
                let parent = joint_name_to_node
                    .get(&j.mimic.joint)
                    .unwrap_or_else(|| panic!("{} not found, mimic not found", &j.mimic.joint));
                child.set_mimic_parent(parent, (&j.mimic).into());
            }
        }
        // set root as parent of root joint nodes
        let root_nodes = ref_nodes
            .iter()
            .filter(|ref_node| ref_node.parent().is_none());
        for rjn in root_nodes {
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

pub fn joint_to_link_map(urdf_robot: &urdf_rs::Robot) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for j in &urdf_robot.joints {
        map.insert(j.name.to_owned(), j.child.link.to_owned());
    }
    for l in &urdf_robot.links {
        if map.get(&l.name).is_none() {
            map.insert(ROOT_JOINT_NAME.to_owned(), l.name.to_owned());
        }
    }
    map
}

#[test]
fn test_tree() {
    let robot = urdf_rs::read_file("urdf/sample.urdf").unwrap();
    assert_eq!(robot.name, "robo");
    assert_eq!(robot.links.len(), 1 + 6 + 6);

    let tree = Chain::<f32>::from(&robot);
    assert_eq!(tree.iter().count(), 13);
}

#[test]
fn test_tree_from_file() {
    let tree = Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    assert_eq!(tree.dof(), 12);
    let names = tree
        .iter()
        .map(|joint| joint.joint().name.clone())
        .collect::<Vec<_>>();
    assert_eq!(names.len(), 13);
    println!("{}", names[0]);
    assert_eq!(names[0], "root");
    assert_eq!(names[1], "r_shoulder_yaw");
}
