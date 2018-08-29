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
//! // read the sample urdf file and create `LinkTree` instance.
//! let robot = k::LinkTree::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
//!
//! // Create `Manipulator` instance to use inverse kinematics.
//! // "l_wrist2" is the name of the end link. The joints between "l_wrist2" and
//! // the root link is used to solve IK.
//! let mut arm = k::Manipulator::from_link_tree("l_wrist2", &robot).unwrap();
//!
//! // Set joint angles
//! let angles = vec![0.5, 0.2, 0.0, -1.0, 0.0, -0.3];
//! arm.set_joint_angles(&angles).unwrap();
//! println!("initial angles={:?}", arm.joint_angles());
//!
//! // Get the transform of the end of the manipulator (forward kinematics)
//! let mut target = arm.end_transform();
//!
//! println!("initial target pos = {}", target.translation);
//! println!("move x: -0.1");
//! target.translation.vector.x -= 0.1;
//!
//! // Create IK solver
//! let solver = k::JacobianIKSolverBuilder::new().finalize();
//!
//! // Solve and move the manipulator joint angles
//! solver.solve(&mut arm, &target).unwrap();
//! println!("solved angles={:?}", arm.joint_angles());
//! println!(
//!     "solved target pos = {}",
//!     arm.end_transform().translation
//! );
//! ```

//!
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
