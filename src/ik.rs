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
use math::*;
use na::{self, DMatrix, Isometry3, Real, Vector6};
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

/// IK solver
pub trait InverseKinematicsSolver<T>
where
    T: Real,
{
    /// Move the end transform of the `arm` to `target_pose`
    fn solve<K>(
        &self,
        arm: &mut K,
        target_link_name: &str,
        target_pose: &Isometry3<T>,
    ) -> Result<T, IKError>
    where
        K: HasJoints<T> + HasLinks<T>;
}

/// Inverse Kinematics Solver using Jacobian matrix
#[derive(Debug, Clone)]
pub struct JacobianIKSolver<T: Real> {
    /// Calculate jacobian matrix with this length in each joint
    pub jacobian_move_epsilon: T,
    /// Move by this length in one loop
    pub move_epsilon: T,
    /// If the distance is smaller than this value, it is reached.
    pub allowable_target_distance: T,
    /// How many times the joints are tried to be moved
    pub num_max_try: usize,
}

impl<T> JacobianIKSolver<T>
where
    T: Real,
{
    /// Create instance of `JacobianIKSolver`.
    ///
    ///  `JacobianIKSolverBuilder` is available instead of calling this `new` method.
    ///
    /// # Examples
    ///
    /// ```
    /// let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.01, 1000);
    /// ```
    pub fn new(
        jacobian_move_epsilon: T,
        move_epsilon: T,
        allowable_target_distance: T,
        num_max_try: usize,
    ) -> JacobianIKSolver<T> {
        JacobianIKSolver {
            jacobian_move_epsilon: jacobian_move_epsilon,
            move_epsilon: move_epsilon,
            allowable_target_distance: allowable_target_distance,
            num_max_try: num_max_try,
        }
    }
    fn solve_one_loop<K>(
        &self,
        arm: &mut K,
        target_link_name: &str,
        target_pose: &Isometry3<T>,
    ) -> Result<T, IKError>
    where
        K: HasJoints<T> + HasLinks<T>,
    {
        let orig_angles = arm.joint_angles();
        let dof = orig_angles.len();
        let orig_poses = arm.update_transforms();
        let pose_index = arm
            .link_names()
            .iter()
            .position(|name| name == target_link_name)
            .ok_or(IKError::InvalidArguments {
                error: format!("{} not found", target_link_name),
            })?;
        let orig_pose6 = calc_vector6_pose(&orig_poses[pose_index]);
        let target_pose6 = calc_vector6_pose(target_pose);
        let mut jacobi_vec = Vec::new();
        for i in 0..dof {
            let mut small_diff_angles_i = orig_angles.clone();
            small_diff_angles_i[i] += self.jacobian_move_epsilon;
            arm.set_joint_angles(&small_diff_angles_i)?;
            let small_diff_pose6 = calc_vector6_pose(&arm.update_transforms()[pose_index]);
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
            jacobi.try_inverse().ok_or(IKError::InverseMatrixError)?
        };
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * self.move_epsilon;
        let mut angles_vec = orig_angles.clone();
        for i in 0..dof {
            angles_vec[i] += new_angles_diff[i];
        }
        arm.set_joint_angles(&angles_vec)?;
        let new_pose6 = calc_vector6_pose(&arm.update_transforms()[pose_index]);
        Ok((target_pose6 - new_pose6).norm())
    }
}

impl<T> InverseKinematicsSolver<T> for JacobianIKSolver<T>
where
    T: Real,
{
    /// Set joint angles of `arm` to reach the `target_pose`
    ///
    /// # Examples
    ///
    /// ```
    /// use k::prelude::*;
    ///
    /// let robot = k::LinkTree::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    /// // Create sub-`LinkTree` to make it easy to use inverse kinematics
    /// let target_link_name = "r_wrist2";
    /// let r_wrist = robot.iter().find(|link| link.is_link_name(target_link_name)).unwrap().clone();
    /// let mut arm = k::LinkTree::from_end("r-arm", r_wrist.clone());
    /// println!("arm: {}", arm);
    ///
    /// // Set joint angles of `arm`
    /// let angles = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    /// arm.set_joint_angles(&angles).unwrap();
    /// println!("initial angles={:?}", arm.joint_angles());
    ///
    /// // Get the transform of the end of the manipulator (forward kinematics)
    /// let mut target = arm.update_transforms().last().unwrap().clone();
    ///
    /// println!("initial target pos = {}", target.translation);
    /// println!("move x: -0.1");
    /// target.translation.vector.x -= 0.1;
    ///
    /// // Create IK solver with default settings
    /// let solver = k::JacobianIKSolverBuilder::new().finalize();
    ///
    /// // solve and move the manipulator angles
    /// solver.solve(&mut arm, target_link_name, &target).unwrap();
    /// println!("solved angles={:?}", arm.joint_angles());
    /// ```
    fn solve<K>(
        &self,
        arm: &mut K,
        target_link_name: &str,
        target_pose: &Isometry3<T>,
    ) -> Result<T, IKError>
    where
        K: HasLinks<T> + HasJoints<T>,
    {
        let orig_angles = arm.joint_angles();
        if orig_angles.len() < 6 {
            println!("support only 6 or more DoF now");
            return Err(IKError::PreconditionError {
                error: format!(
                    "support only 6 or more DoF now, input Dof={}",
                    orig_angles.len()
                ),
            });
        }
        let mut last_target_distance = None;
        for _ in 0..self.num_max_try {
            let target_distance = self.solve_one_loop(arm, target_link_name, target_pose)?;
            if target_distance < self.allowable_target_distance {
                return Ok(target_distance);
            }
            if let Some(last) = last_target_distance {
                if last < target_distance {
                    arm.set_joint_angles(&orig_angles)?;
                    return Err(IKError::NotConverged {
                        error: format!("jacobian did not work"),
                    });
                }
            }
            last_target_distance = Some(target_distance);
        }
        arm.set_joint_angles(&orig_angles)?;
        Err(IKError::NotConverged {
            error: format!(
                "iteration has not converged: tried {} timed",
                self.num_max_try
            ),
        })
    }
}

/// Build `jacobianIKSolverBuilder`
///
/// This builder allow initialation of `JacobianIKSolver`
/// without any parameters.
///
#[derive(Debug, Clone)]
pub struct JacobianIKSolverBuilder<T>
where
    T: Real,
{
    pub jacobian_move_epsilon: T,
    pub move_epsilon: T,
    pub allowable_target_distance: T,
    pub num_max_try: usize,
}

impl<T> JacobianIKSolverBuilder<T>
where
    T: Real,
{
    const DEFAULT_JACOBIAN_MOVE_EPSILON: f64 = 0.0001;
    const DEFAULT_MOVE_EPSILON: f64 = 0.0001;
    const DEFAULT_ALLOWABLE_TARGET_DISTANCE: f64 = 0.001;
    const DEAULT_NUM_MAX_TRY: usize = 1000;

    pub fn new() -> Self {
        JacobianIKSolverBuilder {
            jacobian_move_epsilon: na::convert(Self::DEFAULT_JACOBIAN_MOVE_EPSILON),
            move_epsilon: na::convert(Self::DEFAULT_MOVE_EPSILON),
            allowable_target_distance: na::convert(Self::DEFAULT_ALLOWABLE_TARGET_DISTANCE),
            num_max_try: Self::DEAULT_NUM_MAX_TRY,
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
    pub fn num_max_try(&mut self, max_try: usize) -> &mut Self {
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

impl<T> Default for JacobianIKSolverBuilder<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}
