//! # Kinematics(forward/inverse) library using [nalgebra](http://nalgebra.org).
//!
//! `k` has below functionalities
//!
//! 1. Forward kinematics
//! 1. Inverse kinematics
//! 1. URDF Loader
//!
//! ## Forward Kinematics
//!
//! If you deal robot arm without any branches, you can use `VecKinematicChain`,
//! which is just a Vec of `Link`. If you need to deal more complexed
//! link structure, you have two choices now.
//!
//! `k` has two representation for kinematic chain.
//!
//! 1. `LinkTree`
//! 1. `LinkStar`
//!
//! ### `LinkTree`
//!
//! `LinkTree` uses `Rc<RefCell<Node<T>>>` to handle tree structure.
//! It can deal complete tree sctuctures, but has lost the safety of rust,
//! and it has runtime costs.
//!
//! ### `LinkStar`
//!
//! `LinkStar` has very simple structure, Vec of Vec of Link.
//! This is thread safe and fast, but it is not a tree, it has star structure.
//! It means that it can deal humanoid with four limbs without fingers,
//! but it cannot have branches in the limbs like fingers. Only root has branches.
//!
//!
//! # Examples
//!
//! Build `VecKinematicChain` using `LinkBuilder` at first.
//! Instead of using the builder, You can use `URDF` format
//! by `urdf` module if you want.
//!
//! ```
//! extern crate k;
//! extern crate nalgebra;
//!
//! use k::{VecKinematicChain, LinkBuilder, JointType,
//!                   KinematicChain, JacobianIKSolverBuilder,
//!                   InverseKinematicsSolver};
//! use nalgebra::{Vector3, Translation3};
//!
//! fn main() {
//!   let l0 = LinkBuilder::new()
//!       .name("shoulder_link1")
//!       .joint("shoulder_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .finalize();
//!   let l1 = LinkBuilder::new()
//!       .name("shoulder_link2")
//!       .joint("shoulder_roll",
//!              JointType::Rotational { axis: Vector3::x_axis() })
//!       .translation(Translation3::new(0.0, 0.1, 0.0))
//!       .finalize();
//!   let l2 = LinkBuilder::new()
//!       .name("shoulder_link3")
//!       .joint("shoulder_yaw",
//!              JointType::Rotational { axis: Vector3::z_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.30))
//!       .finalize();
//!   let l3 = LinkBuilder::new()
//!       .name("elbow_link1")
//!       .joint("elbow_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l4 = LinkBuilder::new()
//!       .name("wrist_link1")
//!       .joint("wrist_yaw",
//!              JointType::Rotational { axis: Vector3::z_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l5 = LinkBuilder::new()
//!       .name("wrist_link2")
//!       .joint("wrist_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l6 = LinkBuilder::new()
//!       .name("wrist_link3")
//!       .joint("wrist_roll",
//!              JointType::Rotational { axis: Vector3::x_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.10))
//!       .finalize();
//!   let mut arm = VecKinematicChain::new("arm", vec![l0, l1, l2, l3, l4, l5, l6]);
//!
//!   // set joint angles
//!   let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
//!   arm.set_joint_angles(&angles).unwrap();
//!   // get the transform of the end of the manipulator (forward kinematics)
//!   let mut target = arm.calc_end_transform();
//!   target.translation.vector[2] += 0.1;
//!   let solver = JacobianIKSolverBuilder::new().finalize();
//!   // solve and move the manipulator angles
//!   solver.solve(&mut arm, &target)
//!         .unwrap_or_else(|err| {
//!                               println!("Err: {}", err);
//!                               0.0f32
//!                               });
//!   println!("angles={:?}", arm.get_joint_angles());
//! }
//! ```
//!
//! or you can use urdf module to load URDF file.
//!
//! ```
//! let _ = k::urdf::create_tree_from_file::<f32, _>("urdf/sample.urdf").unwrap();
//! ```
extern crate alga;
extern crate nalgebra as na;
#[macro_use]
extern crate log;

mod links;
mod ik;
pub mod math;
pub mod rctree;
mod rctree_links;
pub mod urdf;

pub use self::links::*;
pub use self::ik::*;
pub use self::rctree_links::*;
