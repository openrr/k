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
use na::{self, DMatrix, Isometry3, Real, Vector3, Vector6};

use chain::*;
use errors::*;
use joint::*;

/// From 'Humanoid Robot (Kajita)' P.64
fn calc_pose_diff<T>(a: &Isometry3<T>, b: &Isometry3<T>) -> Vector6<T>
where
    T: Real,
{
    let p_diff = a.translation.vector - b.translation.vector;
    let w_diff = b.rotation.rotation_to(&a.rotation).scaled_axis();
    Vector6::new(
        p_diff[0], p_diff[1], p_diff[2], w_diff[0], w_diff[1], w_diff[2],
    )
}

/// IK solver
pub trait InverseKinematicsSolver<T>
where
    T: Real,
{
    /// Move the end transform of the `arm` to `target_pose`
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), IKError>;
}

/// Inverse Kinematics Solver using Jacobian matrix
#[derive(Debug, Clone)]
pub struct JacobianIKSolver<T: Real> {
    /// If the distance is smaller than this value, it is reached.
    pub allowable_target_distance: T,
    /// Weight for rotation
    pub allowable_target_angle: T,
    /// multiplier for jacobian
    pub jacobian_multiplier: T,
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
    /// let solver = k::JacobianIKSolver::new(0.01, 0.01, 0.5, 100);
    /// ```
    pub fn new(
        allowable_target_distance: T,
        allowable_target_angle: T,
        jacobian_multiplier: T,
        num_max_try: usize,
    ) -> JacobianIKSolver<T> {
        JacobianIKSolver {
            allowable_target_distance,
            allowable_target_angle,
            jacobian_multiplier,
            num_max_try,
        }
    }
    fn add_positions_with_multiplier(&self, input: &[T], add_values: &[T]) -> Vec<T> {
        input
            .iter()
            .zip(add_values.iter())
            .map(|(ang, add)| *ang + self.jacobian_multiplier * *add)
            .collect()
    }

    fn solve_one_loop(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
    ) -> Result<Vector6<T>, IKError> {
        let orig_positions = arm.joint_positions();
        let dof = orig_positions.len();
        let t_n = arm.end_transform();
        let err = calc_pose_diff(target_pose, &t_n);
        arm.update_transforms();
        let p_n = t_n.translation;
        let jacobi_vec = arm
            .iter_joints()
            .map(|joint| {
                let t_i = joint.world_transform().unwrap();
                let p_i = t_i.translation;
                let a_i = t_i.rotation * match joint.joint_type {
                    JointType::Linear { axis } => axis,
                    JointType::Rotational { axis } => axis,
                    JointType::Fixed => panic!("impossible, bug of jacobian"),
                };
                let dp_i = a_i.cross(&(p_n.vector - p_i.vector));
                [dp_i[0], dp_i[1], dp_i[2], a_i[0], a_i[1], a_i[2]]
            }).collect::<Vec<_>>();
        // Pi: a_i x (p_n - Pi)
        // wi: a_i
        let jacobi = DMatrix::from_fn(6, dof, |r, c| jacobi_vec[c][r]);
        let positions_vec = if dof > 6 {
            self.add_positions_with_multiplier(
                &orig_positions,
                jacobi
                    .svd(true, true)
                    .solve(&err, na::convert(0.0001))
                    .as_slice(),
            )
        } else {
            self.add_positions_with_multiplier(
                &orig_positions,
                &jacobi
                    .lu()
                    .solve(&err)
                    .ok_or(IKError::InverseMatrixError)?
                    .as_slice(),
            )
        };
        arm.set_joint_positions(&positions_vec)?;
        Ok(calc_pose_diff(target_pose, &arm.end_transform()))
    }
}

impl<T> InverseKinematicsSolver<T> for JacobianIKSolver<T>
where
    T: Real,
{
    /// Set joint positions of `arm` to reach the `target_pose`
    ///
    /// # Examples
    ///
    /// ```
    /// use k::prelude::*;
    ///
    /// let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    /// // Create sub-`Chain` to make it easy to use inverse kinematics
    /// let target_joint_name = "r_wrist_pitch";
    /// let r_wrist = chain.find(target_joint_name).unwrap();
    /// let mut arm = k::SerialChain::from_end(r_wrist);
    /// println!("arm: {}", arm);
    ///
    /// // Set joint positions of `arm`
    /// let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    /// arm.set_joint_positions(&positions).unwrap();
    /// println!("initial positions={:?}", arm.joint_positions());
    ///
    /// // Get the transform of the end of the manipulator (forward kinematics)
    /// let mut target = arm.update_transforms().last().unwrap().clone();
    ///
    /// println!("initial target pos = {}", target.translation);
    /// println!("move x: -0.1");
    /// target.translation.vector.x -= 0.1;
    ///
    /// // Create IK solver with default settings
    /// let solver = k::JacobianIKSolver::default();
    ///
    /// // solve and move the manipulator positions
    /// solver.solve(&arm, &target).unwrap();
    /// println!("solved positions={:?}", arm.joint_positions());
    /// ```
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), IKError> {
        let orig_positions = arm.joint_positions();
        if orig_positions.len() < 6 {
            return Err(IKError::PreconditionError {
                error: format!(
                    "support only 6 or more DoF now, input Dof={}",
                    orig_positions.len()
                ),
            });
        }
        let mut last_target_distance = None;
        for _ in 0..self.num_max_try {
            let target_diff = self.solve_one_loop(&arm, target_pose)?;
            let len_diff = Vector3::new(target_diff[0], target_diff[1], target_diff[2]).norm();
            let rot_diff = Vector3::new(target_diff[3], target_diff[4], target_diff[5]).norm();
            if len_diff < self.allowable_target_distance && rot_diff < self.allowable_target_angle {
                return Ok(());
            }
            if let Some((last_len, last_rot)) = last_target_distance {
                if last_len < len_diff && last_rot < rot_diff {
                    arm.set_joint_positions(&orig_positions)?;
                    return Err(IKError::NotConvergedError {
                        error: "jacobian did not work".to_owned(),
                    });
                }
            }
            last_target_distance = Some((len_diff, rot_diff));
        }
        arm.set_joint_positions(&orig_positions)?;
        Err(IKError::NotConvergedError {
            error: format!(
                "iteration has not converged: tried {} timed, diff = {}, {}",
                self.num_max_try,
                last_target_distance.unwrap().0,
                last_target_distance.unwrap().1,
            ),
        })
    }
}

impl<T> Default for JacobianIKSolver<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new(na::convert(0.001), na::convert(0.005), na::convert(0.5), 10)
    }
}
