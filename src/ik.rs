extern crate nalgebra as na;

use na::{Isometry3, Vector6, DMatrix, Real};
use errors::*;
use math::*;
use traits::*;

fn calc_vector6_pose<T: Real>(pose: &Isometry3<T>) -> Vector6<T> {
    let rpy = to_euler_angles(&pose.rotation);
    Vector6::new(
        pose.translation.vector[0],
        pose.translation.vector[1],
        pose.translation.vector[2],
        rpy[0],
        rpy[1],
        rpy[2],
    )
}

pub trait InverseKinematicsSolver<T: Real> {
    fn solve<K>(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError>
    where
        K: KinematicChain<T>;
}


/// Inverse Kinematics Solver using Jacobian matrix
#[derive(Debug)]
pub struct JacobianIKSolver<T: Real> {
    pub jacobian_move_epsilon: T,
    pub move_epsilon: T,
    pub allowable_target_distance: T,
    pub num_max_try: i32,
}


impl<T> Default for JacobianIKSolverBuilder<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> JacobianIKSolver<T>
where
    T: Real,
{
    pub fn new(
        jacobian_move_epsilon: T,
        move_epsilon: T,
        allowable_target_distance: T,
        num_max_try: i32,
    ) -> JacobianIKSolver<T> {
        JacobianIKSolver {
            jacobian_move_epsilon: jacobian_move_epsilon,
            move_epsilon: move_epsilon,
            allowable_target_distance: allowable_target_distance,
            num_max_try: num_max_try,
        }
    }
    fn solve_one_loop<K>(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError>
    where
        K: KinematicChain<T>,
    {
        let orig_angles = arm.get_joint_angles();
        let dof = orig_angles.len();
        let orig_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        let target_pose6 = calc_vector6_pose(target_pose);
        let mut jacobi_vec = Vec::new();
        for i in 0..dof {
            let mut small_diff_angles_i = orig_angles.clone();
            small_diff_angles_i[i] += self.jacobian_move_epsilon;
            try!(arm.set_joint_angles(&small_diff_angles_i));
            let small_diff_pose6 = calc_vector6_pose(&arm.calc_end_transform());
            jacobi_vec.push(small_diff_pose6 - orig_pose6);
        }
        let jacobi = DMatrix::from_fn(6, dof, |r, c| jacobi_vec[c][r]);
        let j_inv = if dof > 6 {
            let jacobi_ref = &jacobi;
            // use pseudo inverse
            match (jacobi_ref * jacobi_ref.transpose()).try_inverse() {
                Some(mat) => jacobi_ref.transpose() * mat,
                None => return Err(IKError::InverseMatrixError),
            }
        } else {
            try!(jacobi.try_inverse().ok_or(IKError::InverseMatrixError))
        };
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * self.move_epsilon;
        let mut angles_vec = Vec::new();
        for i in 0..dof {
            angles_vec.push(orig_angles[i] + new_angles_diff[i]);
        }
        try!(arm.set_joint_angles(&angles_vec));
        let new_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        Ok((target_pose6 - new_pose6).norm())
    }
}

impl<T> InverseKinematicsSolver<T> for JacobianIKSolver<T>
where
    T: Real,
{
    fn solve<K>(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError>
    where
        K: KinematicChain<T>,
    {
        let orig_angles = arm.get_joint_angles();
        if orig_angles.len() < 6 {
            println!("support only 6 or more DoF now");
            return Err(IKError::PreconditionError);
        }
        for _ in 0..self.num_max_try {
            let target_distance = try!(self.solve_one_loop(arm, target_pose));
            if target_distance < self.allowable_target_distance {
                return Ok(target_distance);
            }
        }
        try!(arm.set_joint_angles(&orig_angles));
        Err(IKError::NotConverged)
    }
}


/// Build `jacobianIKSolverBuilder`
///
/// This builder allow initialation of `JacobianIKSolver`
/// without any parameters.
///
pub struct JacobianIKSolverBuilder<T>
where
    T: Real,
{
    pub jacobian_move_epsilon: T,
    pub move_epsilon: T,
    pub allowable_target_distance: T,
    pub num_max_try: i32,
}

impl<T> JacobianIKSolverBuilder<T>
where
    T: Real,
{
    pub fn new() -> Self {
        JacobianIKSolverBuilder {
            jacobian_move_epsilon: na::convert(0.001),
            move_epsilon: na::convert(0.001),
            allowable_target_distance: na::convert(0.001),
            num_max_try: 100,
        }
    }
    pub fn jacobian_move_epsilon(&mut self, jacobian_epsilon: T) -> &mut Self {
        self.jacobian_move_epsilon = jacobian_epsilon;
        self
    }
    pub fn move_epsilon(&mut self, epsilon: T) -> &mut Self {
        self.move_epsilon = epsilon;
        self
    }
    pub fn allowable_target_distance(&mut self, allowable_diff: T) -> &mut Self {
        self.allowable_target_distance = allowable_diff;
        self
    }
    pub fn num_max_try(&mut self, max_try: i32) -> &mut Self {
        self.num_max_try = max_try;
        self
    }
    pub fn finalize(&self) -> JacobianIKSolver<T> {
        JacobianIKSolver {
            jacobian_move_epsilon: self.jacobian_move_epsilon,
            move_epsilon: self.move_epsilon,
            allowable_target_distance: self.allowable_target_distance,
            num_max_try: self.num_max_try,
        }
    }
}
