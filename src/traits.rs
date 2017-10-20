use errors::*;
use na::{Isometry3, Real};

pub trait JointContainer<T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError>;
    fn get_joint_angles(&self) -> Vec<T>;
}

pub trait KinematicChain<T>: JointContainer<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T>;
}
