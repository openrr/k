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
//! # Kinematics(forward/inverse) library using [nalgebra](http://nalgebra.org).
//!
//! `k` has below functionalities
//!
//! 1. Forward kinematics
//! 1. Inverse kinematics
//! 1. URDF Loader
//!
//! # Examples
//!
//! ```
//! use k::prelude::*;
//!
//! // Load urdf file
//! let robot = k::LinkTree::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
//! println!("robot: {}", robot);
//!
//! // Create sub-`LinkTree` to make it easy to use inverse kinematics
//! let target_link_name = "r_wrist2";
//! let r_wrist = robot
//!     .iter()
//!     .find(|link| link.is_link_name(target_link_name))
//!     .unwrap()
//!     .clone();
//! let mut arm = k::LinkTree::from_end("r-arm", r_wrist);
//! println!("arm: {}", arm);
//!
//! // Set joint angles of `arm`
//! let angles = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
//! arm.set_joint_angles(&angles).unwrap();
//! println!("initial angles={:?}", arm.joint_angles());
//!
//! // Get the transform of the end of the manipulator (forward kinematics)
//! let mut target = arm.update_transform_with_name(target_link_name).unwrap();
//!
//! println!("initial target pos = {}", target.translation);
//! println!("move x: -0.1");
//! target.translation.vector.x -= 0.1;
//!
//! // Create IK solver with default settings
//! let solver = k::JacobianIKSolverBuilder::new().finalize();
//!
//! // solve and move the manipulator angles
//! solver.solve(&mut arm, target_link_name, &target).unwrap();
//! println!("solved angles={:?}", arm.joint_angles());
//!
//! let solved_pose = arm.update_transform_with_name(target_link_name).unwrap();
//! println!("solved target pos = {}", solved_pose.translation);
//! ```
#[macro_use]
extern crate failure;
#[macro_use]
extern crate log;
extern crate nalgebra as na;
extern crate urdf_rs;

mod errors;
mod ik;
mod joints;
mod links;
mod rctree;
mod rctree_links;
mod traits;

pub mod math;
pub mod prelude;
pub mod urdf;

pub use self::errors::*;
pub use self::ik::*;
pub use self::joints::*;
pub use self::links::*;
pub use self::rctree::Node;
pub use self::rctree_links::*;
pub use self::traits::*;
