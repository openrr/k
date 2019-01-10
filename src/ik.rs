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
use na::{self, DVector, Isometry3, Real, Vector3, Vector6};

use chain::*;
use errors::*;
use funcs::*;

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

fn calc_pose_diff_with_constraints<T>(
    a: &Isometry3<T>,
    b: &Isometry3<T>,
    constraints_array: &[bool; 6],
) -> DVector<T>
where
    T: Real,
{
    let full_diff = calc_pose_diff(a, b);
    let use_dof = constraints_array.into_iter().filter(|x| **x).count();
    let mut diff = DVector::from_element(use_dof, na::zero());
    let mut index = 0;
    for (i, use_i) in constraints_array.iter().enumerate() {
        if *use_i {
            diff[index] = full_diff[i];
            index += 1;
        }
    }
    diff
}

pub struct Constraints {
    pub position_x: bool,
    pub position_y: bool,
    pub position_z: bool,
    pub rotation_x: bool,
    pub rotation_y: bool,
    pub rotation_z: bool,
}

fn constraints_to_bool_array(constraints: &Constraints) -> [bool; 6] {
    let mut arr = [true; 6];
    arr[0] = constraints.position_x;
    arr[1] = constraints.position_y;
    arr[2] = constraints.position_z;
    arr[3] = constraints.rotation_x;
    arr[4] = constraints.rotation_y;
    arr[5] = constraints.rotation_z;
    arr
}

/// IK solver
pub trait InverseKinematicsSolver<T>
where
    T: Real,
{
    /// Move the end transform of the `arm` to `target_pose`
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), IKError>;
}

/// constraints configurable IK solver
pub trait ConstraintsConfigurableInverseKinematicsSolver<T>
where
    T: Real,
{
    fn solve_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), IKError>;
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
        let orig_positions = arm.joint_positions();
        let jacobi = jacobian(arm);
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
        arm.set_joint_positions_unchecked(&positions_vec);
        Ok(calc_pose_diff(target_pose, &arm.end_transform()))
    }
    fn solve_one_loop_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints_array: &[bool; 6],
    ) -> Result<DVector<T>, IKError> {
        let orig_positions = arm.joint_positions();
        let dof = orig_positions.len();
        let t_n = arm.end_transform();
        let err = calc_pose_diff_with_constraints(target_pose, &t_n, constraints_array);
        let orig_positions = arm.joint_positions();
        let mut jacobi = jacobian(arm);
        let use_dof = constraints_array.into_iter().filter(|x| **x).count();
        let mut removed_count = 0;
        for (i, use_i) in constraints_array.iter().enumerate() {
            if !use_i {
                jacobi = jacobi.remove_row(i - removed_count);
                removed_count += 1;
            }
        }
        let positions_vec = if dof > use_dof {
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
        arm.set_joint_positions_unchecked(&positions_vec);
        Ok(calc_pose_diff_with_constraints(
            target_pose,
            &arm.end_transform(),
            constraints_array,
        ))
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
                let non_checked_positions = arm.joint_positions();
                return Ok(arm.set_joint_positions(&non_checked_positions)?);
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

impl<T> ConstraintsConfigurableInverseKinematicsSolver<T> for JacobianIKSolver<T>
where
    T: Real,
{
    fn solve_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), IKError> {
        let constraints_array = constraints_to_bool_array(constraints);
        let orig_positions = arm.joint_positions();
        let use_dof = constraints_array.into_iter().filter(|x| **x).count();
        if orig_positions.len() < use_dof {
            return Err(IKError::PreconditionError {
                error: format!(
                    "Input Dof={}, must be greater than {}",
                    orig_positions.len(),
                    use_dof
                ),
            });
        }
        let mut last_target_distance = None;
        for _ in 0..self.num_max_try {
            let target_diff =
                self.solve_one_loop_with_constraints(&arm, target_pose, &constraints_array)?;
            let mut len_diff = Vector3::zeros();
            let mut rot_diff = Vector3::zeros();
            let mut index = 0;
            if constraints.position_x {
                len_diff[0] = target_diff[index];
                index += 1;
            }
            if constraints.position_y {
                len_diff[1] = target_diff[index];
                index += 1;
            }
            if constraints.position_z {
                len_diff[2] = target_diff[index];
                index += 1;
            }
            if constraints.rotation_x {
                rot_diff[0] = target_diff[index];
                index += 1;
            }
            if constraints.rotation_y {
                rot_diff[1] = target_diff[index];
                index += 1;
            }
            if constraints.rotation_z {
                rot_diff[2] = target_diff[index];
            }
            if len_diff.norm() < self.allowable_target_distance
                && rot_diff.norm() < self.allowable_target_angle
            {
                let non_checked_positions = arm.joint_positions();
                return Ok(arm.set_joint_positions(&non_checked_positions)?);
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

impl Default for Constraints {
    fn default() -> Self {
        Self {
            position_x: true,
            position_y: true,
            position_z: true,
            rotation_x: true,
            rotation_y: true,
            rotation_z: true,
        }
    }
}
