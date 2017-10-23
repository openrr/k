use errors::*;
use joints::*;
use na::{Isometry3, Real};

pub trait JointContainer<T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError>;
    fn get_joint_angles(&self) -> Vec<T>;
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>>;
    fn get_joint_names(&self) -> Vec<String>;
}

pub trait LinkContainer<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>>;
    fn get_link_names(&self) -> Vec<String>;
}

pub trait KinematicChain<T>: JointContainer<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T>;
}
