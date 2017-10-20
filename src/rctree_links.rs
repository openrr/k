extern crate nalgebra as na;

use na::{Isometry3, Real};
use errors::*;
use traits::*;
use links::*;
use rctree::*;
use std::collections::HashSet;
use std::slice::{Iter, IterMut};
use std::rc::Rc;
use std::cell::{Ref, RefMut, RefCell};

pub type RcLinkNode<T> = RcNode<Link<T>>;
pub type LinkNode<T> = Node<Link<T>>;

/// Kinematic chain using `Rc<RefCell<LinkNode<T>>>`
pub struct RefKinematicChain<T: Real> {
    pub name: String,
    pub joint_with_links: Vec<RcLinkNode<T>>,
    pub transform: Isometry3<T>,
}

impl<T> RefKinematicChain<T>
where
    T: Real,
{
    pub fn new(name: &str, end: &RcLinkNode<T>) -> Self {
        let mut links = map_ancestors(end, &|ljn| ljn.clone());
        links.reverse();
        RefKinematicChain {
            name: name.to_string(),
            joint_with_links: links,
            transform: Isometry3::identity(),
        }
    }
}

impl<T> KinematicChain<T> for RefKinematicChain<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.joint_with_links.iter().fold(self.transform, |trans,
         ljn_ref| {
            trans * ljn_ref.borrow().data.calc_transform()
        })
    }
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut joints_with_angle = self.joint_with_links
            .iter_mut()
            .filter(|ljn_ref| ljn_ref.borrow().data.has_joint_angle())
            .collect::<Vec<_>>();
        if joints_with_angle.len() != angles.len() {
            return Err(JointError::SizeMisMatch);
        }
        for (i, ljn_ref) in joints_with_angle.iter_mut().enumerate() {
            try!(ljn_ref.borrow_mut().data.set_joint_angle(angles[i]));
        }
        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.joint_with_links
            .iter()
            .filter_map(|ljn_ref| ljn_ref.borrow().data.get_joint_angle())
            .collect()
    }
}

pub struct NodeIter<'a, T: 'a> {
    iter: Iter<'a, Rc<RefCell<Node<T>>>>,
}

impl<'a, T: 'a> Iterator for NodeIter<'a, T> {
    type Item = Ref<'a, T>;

    fn next(&mut self) -> Option<Ref<'a, T>> {
        self.iter.next().map(|rc| {
            Ref::map(rc.borrow(), |node| &node.data)
        })
    }
}

pub struct NodeIterMut<'a, T: 'a> {
    iter: Iter<'a, Rc<RefCell<Node<T>>>>,
}

impl<'a, T: 'a> Iterator for NodeIterMut<'a, T> {
    type Item = RefMut<'a, T>;

    fn next(&mut self) -> Option<RefMut<'a, T>> {
        self.iter.next().map(|rc| {
            RefMut::map(rc.borrow_mut(), |node| &mut node.data)
        })
    }
}

/// Kinematic Tree using `Rc<RefCell<Link<T>>>`
pub struct LinkTree<T: Real> {
    pub name: String,
    pub root_link: RcLinkNode<T>,
    expanded_robot_link_vec: Vec<RcLinkNode<T>>,
}

impl<T: Real> LinkTree<T> {
    pub fn new(name: &str, root_link: RcLinkNode<T>) -> Self {
        LinkTree {
            name: name.to_string(),
            expanded_robot_link_vec: map_descendants(&root_link, &|ln| ln.clone()),
            root_link: root_link,
        }
    }
    pub fn set_root_transform(&mut self, transform: Isometry3<T>) {
        self.root_link.borrow_mut().data.transform = transform;
    }
    pub fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.iter()
            .map(|ljn| {
                let parent_transform = match ljn.borrow().parent {
                    Some(ref parent) => {
                        let rc_parent = parent.upgrade().unwrap().clone();
                        let parent_obj = rc_parent.borrow();
                        match parent_obj.data.world_transform_cache {
                            Some(trans) => trans,
                            None => Isometry3::identity(),
                        }
                    }
                    None => Isometry3::identity(),
                };
                let trans = parent_transform * ljn.borrow().data.calc_transform();
                ljn.borrow_mut().data.world_transform_cache = Some(trans);
                trans
            })
            .collect()
    }

    pub fn iter(&self) -> Iter<RcLinkNode<T>> {
        self.expanded_robot_link_vec.iter()
    }
    pub fn iter_mut(&mut self) -> IterMut<RcLinkNode<T>> {
        self.expanded_robot_link_vec.iter_mut()
    }
    pub fn iter_link<'a>(&'a self) -> NodeIter<'a, Link<T>> {
        NodeIter { iter: self.expanded_robot_link_vec.iter() }
    }
    pub fn iter_link_mut<'a>(&'a self) -> NodeIterMut<'a, Link<T>> {
        NodeIterMut { iter: self.expanded_robot_link_vec.iter() }
    }

    pub fn iter_for_joints<'a>(&'a self) -> Box<Iterator<Item = &RcLinkNode<T>> + 'a> {
        Box::new(self.iter().filter(
            |ljn| ljn.borrow().data.has_joint_angle(),
        ))
    }
    pub fn iter_for_joints_link<'a>(&'a self) -> Box<Iterator<Item = Ref<'a, Link<T>>> + 'a> {
        Box::new(self.iter_link().filter(|link| link.has_joint_angle()))
    }

    /// get the angles of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    pub fn get_joint_angles(&self) -> Vec<T> {
        self.iter_link()
            .filter_map(|link| link.get_joint_angle())
            .collect()
    }

    /// set the angles of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    pub fn set_joint_angles(&mut self, angles_vec: &[T]) -> Result<(), JointError> {
        if angles_vec.len() != self.dof() {
            return Err(JointError::SizeMisMatch);
        }
        for (lj, angle) in self.iter_for_joints().zip(angles_vec.iter()) {
            lj.borrow_mut().data.set_joint_angle(*angle)?;
        }
        Ok(())
    }

    /// skip fixed joint
    pub fn get_joint_names(&self) -> Vec<String> {
        self.iter_for_joints_link()
            .map(|link| link.get_joint_name().to_string())
            .collect()
    }

    /// include fixed joint
    pub fn get_all_joint_names(&self) -> Vec<String> {
        self.iter_link()
            .map(|link| link.get_joint_name().to_string())
            .collect()
    }

    /// get the degree of freedom
    pub fn dof(&self) -> usize {
        self.iter_for_joints().count()
    }
}

/// Create `Vec<RefKinematicChain>` from `LinkTree` to use IK
pub fn create_kinematic_chains<T>(tree: &LinkTree<T>) -> Vec<RefKinematicChain<T>>
where
    T: Real,
{
    create_kinematic_chains_with_dof_limit(tree, usize::max_value())
}

/// Create `Vec<RefKinematicChain>` from `LinkTree` to use IK
pub fn create_kinematic_chains_with_dof_limit<T>(
    tree: &LinkTree<T>,
    dof_limit: usize,
) -> Vec<RefKinematicChain<T>>
where
    T: Real,
{
    let mut name_hash = HashSet::new();
    tree.iter()
        .filter_map(|ljn_ref| if ljn_ref.borrow().children.is_empty() {
            Some(ljn_ref.clone())
        } else {
            None
        })
        .filter_map(|ljn_ref| {
            let dof = map_ancestors(&ljn_ref, &|ljn_ref| ljn_ref.clone())
                .iter()
                .filter(|ljn_ref| ljn_ref.borrow().data.has_joint_angle())
                .count();
            let mut root = ljn_ref.clone();
            // use only movable joint as end effector
            while !root.borrow().data.has_joint_angle() {
                match get_parent_rc(&root) {
                    Some(p) => {
                        root = p;
                    }
                    None => {
                        break;
                    }
                }
            }
            if dof > dof_limit {
                let mut remove_dof = dof - dof_limit;
                while remove_dof > 0 {
                    match get_parent_rc(&root) {
                        Some(parent_rc) => {
                            root = parent_rc;
                            if root.borrow().data.has_joint_angle() {
                                remove_dof -= 1;
                            }
                        }
                        None => {
                            break;
                        }
                    }
                }
            }
            if !name_hash.contains(&root.borrow().data.name) {
                let kc = RefKinematicChain::new(&root.borrow().data.name, &root);
                name_hash.insert(root.borrow().data.name.clone());
                assert!(kc.get_joint_angles().len() <= dof_limit);
                if kc.get_joint_angles().len() >= 6 {
                    Some(kc)
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect::<Vec<_>>()
}

#[test]
fn it_works() {
    let l0 = LinkBuilder::new()
        .name("link1")
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
        .name("link1")
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

    let ljn0 = create_ref_node(l0);
    let ljn1 = create_ref_node(l1);
    let ljn2 = create_ref_node(l2);
    let ljn3 = create_ref_node(l3);
    let ljn4 = create_ref_node(l4);
    let ljn5 = create_ref_node(l5);
    set_parent_child(&ljn0, &ljn1);
    set_parent_child(&ljn1, &ljn2);
    set_parent_child(&ljn2, &ljn3);
    set_parent_child(&ljn0, &ljn4);
    set_parent_child(&ljn4, &ljn5);
    let names = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_name().to_string());
    println!("{:?}", ljn0);
    println!("names = {:?}", names);
    let angles = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_angle());
    println!("angles = {:?}", angles);

    let get_z = |ljn: &RcLinkNode<f32>| match ljn.borrow().parent {
        Some(ref parent) => {
            let rc_parent = parent.upgrade().unwrap().clone();
            let parent_obj = rc_parent.borrow();
            (parent_obj.data.calc_transform() * ljn.borrow().data.calc_transform())
                .translation
                .vector
                .z
        }
        None => ljn.borrow().data.calc_transform().translation.vector.z,
    };

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    let _ = map_descendants(&ljn0, &|ljn| ljn.borrow_mut().data.set_joint_angle(-0.5));
    let angles = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_angle());
    println!("angles = {:?}", angles);

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    let arm = RefKinematicChain::new("chain1", &ljn3);
    assert_eq!(arm.get_joint_angles().len(), 4);
    println!("{:?}", arm.get_joint_angles());
    println!("{:?}", arm.calc_end_transform());
}
