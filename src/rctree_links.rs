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
extern crate nalgebra as na;

use std::cell::{Ref, RefCell};
use na::{Isometry3, Real};
use std::slice::{Iter, IterMut};

use errors::*;
use joints::*;
use traits::*;
use links::*;
use rctree::*;

pub type LinkNode<T> = Node<Link<T>>;

/// Kinematic chain using `Rc<RefCell<LinkNode<T>>>`
pub struct LinkChain<T: Real> {
    pub name: String,
    pub links: Vec<LinkNode<T>>,
    pub transform: Isometry3<T>,
    end_link_name: Option<String>,
}

impl<T> LinkChain<T>
where
    T: Real,
{
    pub fn set_end_link_name(&mut self, name: &str) -> Result<(), String> {
        if self.links
            .iter()
            .find(|&ljn| ljn.borrow().data.name == name)
            .is_none()
        {
            Err(format!("{} not found", name).to_owned())
        } else {
            self.end_link_name = Some(name.to_owned());
            Ok(())
        }
    }
    pub fn get_end_link_name<'a>(&'a self) -> &'a Option<String> {
        &self.end_link_name
    }
    pub fn new(name: &str, end: &LinkNode<T>) -> Self {
        let mut links = end.iter_ancestors()
            .map(|ljn| ljn.clone())
            .collect::<Vec<_>>();
        links.reverse();
        LinkChain {
            name: name.to_string(),
            links: links,
            transform: Isometry3::identity(),
            end_link_name: None,
        }
    }
}

impl<T> KinematicChain<T> for LinkChain<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        let mut end_transform = self.transform.clone();
        for ljn_ref in &self.links {
            end_transform *= ljn_ref.borrow().data.calc_transform();
            if let Some(ref end_name) = self.end_link_name {
                if end_name == &ljn_ref.borrow().data.name {
                    return end_transform;
                }
            }
        }
        end_transform
    }
}

impl<T> LinkContainer<T> for LinkChain<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.links
            .iter()
            .scan(self.transform, |base, ljn| {
                *base *= ljn.borrow().data.calc_transform();
                Some(*base)
            })
            .collect()
    }
    fn get_link_names(&self) -> Vec<String> {
        self.links
            .iter()
            .map(|ljn| ljn.borrow().data.name.to_owned())
            .collect()
    }
}

impl<T> JointContainer<T> for LinkChain<T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut links_with_angle = self.links
            .iter_mut()
            .filter(|ljn_ref| ljn_ref.borrow().data.has_joint_angle())
            .collect::<Vec<_>>();
        if links_with_angle.len() != angles.len() {
            println!("angles={:?}", angles);
            return Err(JointError::SizeMisMatch);
        }
        for (i, ljn_ref) in links_with_angle.iter_mut().enumerate() {
            try!(ljn_ref.borrow_mut().data.set_joint_angle(angles[i]));
        }
        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.links
            .iter()
            .filter_map(|ljn_ref| ljn_ref.borrow().data.get_joint_angle())
            .collect()
    }
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>> {
        let links_with_angle = self.links
            .iter()
            .filter(|ljn_ref| ljn_ref.borrow().data.has_joint_angle())
            .collect::<Vec<_>>();
        links_with_angle
            .iter()
            .map(|ljn_ref| ljn_ref.borrow().data.joint.limits.clone())
            .collect()
    }
    /// skip fixed joint
    fn get_joint_names(&self) -> Vec<String> {
        let links_with_angle = self.links
            .iter()
            .filter(|ljn_ref| ljn_ref.borrow().data.has_joint_angle())
            .collect::<Vec<_>>();
        links_with_angle
            .iter()
            .map(|ljn_ref| ljn_ref.borrow().data.joint.name.to_string())
            .collect()
    }
}



/// Kinematic Tree using `Rc<RefCell<Link<T>>>`
pub struct LinkTree<T: Real> {
    pub name: String,
    pub root_link: LinkNode<T>,
    expanded_robot_link_vec: Vec<LinkNode<T>>,
}

impl<T: Real> LinkTree<T> {
    /// Create LinkTree from root link
    ///
    /// # Arguments
    ///
    /// * `root_link` - root node of the links
    pub fn new(name: &str, root_link: LinkNode<T>) -> Self {
        LinkTree {
            name: name.to_string(),
            expanded_robot_link_vec: root_link.iter_descendants().map(|ln| ln.clone()).collect(),
            root_link: root_link,
        }
    }
    /// iter for all link nodes
    pub fn iter(&self) -> Iter<LinkNode<T>> {
        self.expanded_robot_link_vec.iter()
    }
    /// iter for all link nodes as mut
    pub fn iter_mut(&mut self) -> IterMut<LinkNode<T>> {
        self.expanded_robot_link_vec.iter_mut()
    }
    /// iter for all links, not as node
    pub fn iter_link<'a>(&'a self) -> NodeIter<'a, Link<T>> {
        NodeIter { iter: self.expanded_robot_link_vec.iter() }
    }
    /// iter for all links as mut, not as node
    pub fn iter_link_mut<'a>(&'a self) -> NodeIterMut<'a, Link<T>> {
        NodeIterMut { iter: self.expanded_robot_link_vec.iter() }
    }
    /// iter for the links with the joint which is not fixed
    pub fn iter_joints<'a>(&'a self) -> Box<Iterator<Item = &LinkNode<T>> + 'a> {
        Box::new(self.iter().filter(
            |ljn| ljn.borrow().data.has_joint_angle(),
        ))
    }
    /// iter for the links with the joint which is not fixed
    pub fn iter_joints_link<'a>(&'a self) -> Box<Iterator<Item = Ref<'a, Link<T>>> + 'a> {
        Box::new(self.iter_link().filter(|link| link.has_joint_angle()))
    }

    /// Get the degree of freedom
    pub fn dof(&self) -> usize {
        self.iter_joints().count()
    }
}


impl<T> JointContainer<T> for LinkTree<T>
where
    T: Real,
{
    /// Get the angles of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    fn get_joint_angles(&self) -> Vec<T> {
        self.iter_link()
            .filter_map(|link| link.get_joint_angle())
            .collect()
    }

    /// Set the angles of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    fn set_joint_angles(&mut self, angles_vec: &[T]) -> Result<(), JointError> {
        if angles_vec.len() != self.dof() {
            return Err(JointError::SizeMisMatch);
        }
        for (lj, angle) in self.iter_joints().zip(angles_vec.iter()) {
            lj.borrow_mut().data.set_joint_angle(*angle)?;
        }
        Ok(())
    }
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.iter_joints()
            .map(|node| node.borrow().data.joint.limits.clone())
            .collect()
    }
    fn get_joint_names(&self) -> Vec<String> {
        self.iter_joints()
            .map(|node| node.borrow().data.joint.name.clone())
            .collect()
    }
}

impl<T> LinkContainer<T> for LinkTree<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|ljn| {
                let parent_transform = match ljn.borrow().parent {
                    Some(ref parent) => {
                        let rc_parent = parent.upgrade().unwrap().clone();
                        let parent_obj = rc_parent.borrow();
                        let cache = parent_obj.data.world_transform_cache.borrow();
                        match *cache {
                            Some(trans) => trans,
                            None => panic!("cache must exist"),
                        }
                    }
                    None => Isometry3::identity(),
                };
                let trans = parent_transform * ljn.borrow().data.calc_transform();
                ljn.borrow_mut().data.world_transform_cache = RefCell::new(Some(trans));
                trans
            })
            .collect()
    }
    fn get_link_names(&self) -> Vec<String> {
        self.iter_link().map(|link| link.name.to_owned()).collect()
    }
}

impl<T> ChainContainer<LinkChain<T>, T> for LinkTree<T>
where
    T: Real,
{
    /// Create LinkChain from `LinkTree` and the name of the end link
    fn get_chain(&self, end_link_name: &str) -> Option<LinkChain<T>> {
        self.iter()
            .find(|&ljn_ref| ljn_ref.borrow().data.name == end_link_name)
            .map(|ljn| LinkChain::new(end_link_name, ljn))
    }
}

#[test]
fn it_works() {
    let l0 = LinkBuilder::new()
        .name("link0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint(
            "j0",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l1 = LinkBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j1",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l2 = LinkBuilder::new()
        .name("link2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j2",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l3 = LinkBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint(
            "j3",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l4 = LinkBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j4",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l5 = LinkBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j5",
            JointType::Rotational { axis: na::Vector3::y_axis() },
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

    let names = ljn0.iter_descendants()
        .map(|ljn| ljn.borrow().data.get_joint_name().to_string())
        .collect::<Vec<_>>();
    println!("{:?}", ljn0);
    assert_eq!(names.len(), 6);
    println!("names = {:?}", names);
    let angles = ljn0.iter_descendants()
        .map(|ljn| ljn.borrow().data.get_joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    fn get_z(ljn: &LinkNode<f32>) -> f32 {
        match ljn.borrow().parent {
            Some(ref parent) => {
                let rc_parent = parent.upgrade().unwrap().clone();
                let parent_obj = rc_parent.borrow();
                (parent_obj.data.calc_transform() * ljn.borrow().data.calc_transform())
                    .translation
                    .vector
                    .z
            }
            None => ljn.borrow().data.calc_transform().translation.vector.z,
        }
    }

    let poses = ljn0.iter_descendants()
        .map(|ljn| get_z(&ljn))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let _ = ljn0.iter_ancestors()
        .map(|ljn| ljn.borrow_mut().data.set_joint_angle(-0.5))
        .collect::<Vec<_>>();
    let angles = ljn0.iter_descendants()
        .map(|ljn| ljn.borrow().data.get_joint_angle())
        .collect::<Vec<_>>();
    println!("angles = {:?}", angles);

    let poses = ljn0.iter_descendants()
        .map(|ljn| get_z(&ljn))
        .collect::<Vec<_>>();
    println!("poses = {:?}", poses);

    let mut arm = LinkChain::new("chain1", &ljn3);
    assert_eq!(arm.get_joint_angles().len(), 4);
    println!("{:?}", arm.get_joint_angles());
    let real_end = arm.calc_end_transform();
    assert!(arm.get_end_link_name().is_none());
    arm.set_end_link_name("link3").unwrap();
    assert!(arm.set_end_link_name("linkhoge").is_err());
    assert!(arm.get_end_link_name().clone().unwrap() == "link3");
    // not changed if set same end link name
    assert_eq!(real_end, arm.calc_end_transform());
    arm.set_end_link_name("link2").unwrap();
    assert!(arm.get_end_link_name().clone().unwrap() == "link2");
    assert!(real_end != arm.calc_end_transform());

    let tree = LinkTree::new("robo1", ljn0);
    assert_eq!(tree.dof(), 6);

    let none_chain = tree.get_chain("link_nono");
    assert!(none_chain.is_none());

    let some_chain = tree.get_chain("link3");
    assert!(some_chain.is_some());
    assert_eq!(some_chain.unwrap().get_joint_angles().len(), 4);
}
