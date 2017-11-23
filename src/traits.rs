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
    /// Set the angles of the joints
    ///
    /// If the angle are out of the limit, it returns error.
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError>;

    /// Get the angles of the joints
    fn get_joint_angles(&self) -> Vec<T>;

    /// Get the limits of the joints, if it exists.
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>>;

    /// Get the all names of the joints
    fn get_joint_names(&self) -> Vec<String>;
}

/// Container of links which has a transformation
pub trait LinkContainer<T>
where
    T: Real,
{
    /// Calculate the transforms of all of the links
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>>;

    /// Get the names of the links
    fn get_link_names(&self) -> Vec<String>;
}

/// `JointContainer` which has an end transform
pub trait KinematicChain<T>: JointContainer<T>
where
    T: Real,
{
    /// Calculate and return the transform of the end of the links
    fn calc_end_transform(&self) -> Isometry3<T>;
}

/// Create KinematicChain
pub trait ChainContainer<K, T>
where
    K: KinematicChain<T>,
    T: Real,
{
    fn get_chain(&self, end_link_name: &str) -> Option<K>;
}
