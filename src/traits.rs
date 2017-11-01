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
use errors::*;
use joints::*;
use na::{Isometry3, Real};

/// Container of links with joints
pub trait JointContainer<T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError>;
    fn get_joint_angles(&self) -> Vec<T>;
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>>;
    fn get_joint_names(&self) -> Vec<String>;
}

/// Container of links which has a transformation
pub trait LinkContainer<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>>;
    fn get_link_names(&self) -> Vec<String>;
}

/// JointContainer which has an end transform
pub trait KinematicChain<T>: JointContainer<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T>;
}
