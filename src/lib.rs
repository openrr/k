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
