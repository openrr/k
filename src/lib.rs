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

#![doc = include_str!("../README.md")]

mod chain;
mod errors;
mod funcs;
mod ik;

pub mod iterator;
pub mod joint;
pub mod link;
pub mod node;
pub mod prelude;
pub mod urdf;

pub use crate::{
    chain::*,
    errors::*,
    funcs::*,
    ik::*,
    joint::{Joint, JointType},
    link::Link,
    node::{Node, NodeBuilder},
};

// re-export from nalgebra
// include Real for backwards compatibility purposes
// (na::Real used to be the name, so we used to re-export k::Real)
#[doc(no_inline)]
pub use nalgebra::{
    Isometry3, RealField as Real, RealField, Translation3, UnitQuaternion, Vector3,
};
// export everything
pub use nalgebra;
pub use simba;
#[doc(no_inline)]
pub use simba::scalar::{SubsetOf, SupersetOf};
