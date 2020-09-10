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
use na::{DVector, Isometry3, RealField, Vector3, Vector6};
use nalgebra as na;
use simba::scalar::SubsetOf;

use super::chain::*;
use super::errors::*;
use super::funcs::*;

/// From 'Humanoid Robot (Kajita)' P.64
fn calc_pose_diff<T>(a: &Isometry3<T>, b: &Isometry3<T>) -> Vector6<T>
where
    T: RealField,
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
    constraints_array: [bool; 6],
) -> DVector<T>
where
    T: RealField,
{
    let full_diff = calc_pose_diff(a, b);
    let use_dof = constraints_array.iter().filter(|x| **x).count();
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

/// A bundle of flags determining which coordinates are constrained for a target
#[derive(Clone, Copy, Debug)]
pub struct Constraints {
    /// true means the constraint is used.
    ///  The coordinates is the world, not the end of the arm.
    pub position_x: bool,
    pub position_y: bool,
    pub position_z: bool,
    pub rotation_x: bool,
    pub rotation_y: bool,
    pub rotation_z: bool,
}

impl Default for Constraints {
    /// Initialize with all true
    ///
    /// ```
    /// let c = k::Constraints::default();
    /// assert!(c.position_x);
    /// assert!(c.position_y);
    /// assert!(c.position_z);
    /// assert!(c.rotation_x);
    /// assert!(c.rotation_y);
    /// assert!(c.rotation_z);
    /// ```
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

fn constraints_to_bool_array(constraints: Constraints) -> [bool; 6] {
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
    T: RealField,
{
    /// Move the end transform of the `arm` to `target_pose`
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), Error> {
        self.solve_with_constraints(arm, target_pose, &Constraints::default())
    }
    /// Move the end transform of the `arm` to `target_pose` with constraints
    fn solve_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error>;
}

/// Inverse Kinematics Solver using Jacobian matrix
pub struct JacobianIKSolver<T: RealField> {
    /// If the distance is smaller than this value, it is reached.
    pub allowable_target_distance: T,
    /// If the angle distance is smaller than this value, it is reached.
    pub allowable_target_angle: T,
    /// multiplier for jacobian
    pub jacobian_multiplier: T,
    /// How many times the joints are tried to be moved
    pub num_max_try: usize,
    /// Nullspace function for a redundant system
    nullspace_function: Option<Box<dyn Fn(&[T]) -> Vec<T>>>,
}

impl<T> JacobianIKSolver<T>
where
    T: RealField + SubsetOf<f64>,
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
            nullspace_function: None,
        }
    }
    /// Set a null space function for redundant manipulator.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut solver = k::JacobianIKSolver::new(0.01, 0.01, 0.5, 100);
    /// solver.set_nullspace_function(Box::new(
    /// k::create_reference_positions_nullspace_function(
    ///    vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ///    vec![0.1, 0.1, 0.1, 1.0, 0.1, 0.5, 0.0],
    ///    ),
    /// ));
    /// ```
    pub fn set_nullspace_function(&mut self, func: Box<dyn Fn(&[T]) -> Vec<T>>) {
        self.nullspace_function = Some(func);
    }

    /// Clear the null function which is set by `set_nullspace_funtion`.
    pub fn clear_nullspace_function(&mut self) {
        self.nullspace_function = None;
    }

    fn add_positions_with_multiplier(&self, input: &[T], add_values: &[T]) -> Vec<T> {
        input
            .iter()
            .zip(add_values.iter())
            .map(|(ang, add)| *ang + self.jacobian_multiplier * *add)
            .collect()
    }

    fn solve_one_loop_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints_array: [bool; 6],
    ) -> Result<DVector<T>, Error> {
        let orig_positions = arm.joint_positions();
        let dof = orig_positions.len();
        let t_n = arm.end_transform();
        let err = calc_pose_diff_with_constraints(target_pose, &t_n, constraints_array);
        let orig_positions = arm.joint_positions();
        let mut jacobi = jacobian(arm);
        let use_dof = constraints_array.iter().filter(|x| **x).count();
        let mut removed_count = 0;
        for (i, use_i) in constraints_array.iter().enumerate() {
            if !use_i {
                jacobi = jacobi.remove_row(i - removed_count);
                removed_count += 1;
            }
        }
        let positions_vec = if dof > use_dof {
            const EPS: f64 = 0.0001;
            // redundant: pseudo inverse
            match self.nullspace_function {
                Some(ref f) => {
                    let jacobi_inv = jacobi.clone().pseudo_inverse(na::convert(EPS)).unwrap();
                    let d_q = jacobi_inv.clone() * err
                        + (na::DMatrix::identity(dof, dof) - jacobi_inv * jacobi)
                            * na::DVector::from_vec(f(&orig_positions));
                    self.add_positions_with_multiplier(&orig_positions, d_q.as_slice())
                }
                None => self.add_positions_with_multiplier(
                    &orig_positions,
                    jacobi
                        .svd(true, true)
                        .solve(&err, na::convert(EPS))
                        .unwrap() // TODO
                        .as_slice(),
                ),
            }
        } else {
            // normal inverse matrix
            self.add_positions_with_multiplier(
                &orig_positions,
                jacobi
                    .lu()
                    .solve(&err)
                    .ok_or(Error::InverseMatrixError)?
                    .as_slice(),
            )
        };
        arm.set_joint_positions_clamped(&positions_vec);
        Ok(calc_pose_diff_with_constraints(
            target_pose,
            &arm.end_transform(),
            constraints_array,
        ))
    }

    fn solve_with_constraints_internal(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        let constraints_array = constraints_to_bool_array(*constraints);
        let orig_positions = arm.joint_positions();
        let use_dof = constraints_array.iter().filter(|x| **x).count();
        if orig_positions.len() < use_dof {
            return Err(Error::PreconditionError {
                dof: orig_positions.len(),
                necessary_dof: use_dof,
            });
        }
        let mut last_target_distance = None;
        for _ in 0..self.num_max_try {
            let target_diff =
                self.solve_one_loop_with_constraints(&arm, target_pose, constraints_array)?;
            let (len_diff, rot_diff) = target_diff_to_len_rot_diff(&target_diff, constraints_array);
            if len_diff.norm() < self.allowable_target_distance
                && rot_diff.norm() < self.allowable_target_angle
            {
                let non_checked_positions = arm.joint_positions();
                arm.set_joint_positions_clamped(&non_checked_positions);
                return Ok(());
            }
            last_target_distance = Some((len_diff, rot_diff));
        }
        arm.set_joint_positions(&orig_positions)?;
        Err(Error::NotConvergedError {
            num_tried: self.num_max_try,
            position_diff: na::convert(last_target_distance.unwrap().0),
            rotation_diff: na::convert(last_target_distance.unwrap().1),
        })
    }
}

fn target_diff_to_len_rot_diff<T>(
    target_diff: &DVector<T>,
    constraints_array: [bool; 6],
) -> (Vector3<T>, Vector3<T>)
where
    T: RealField,
{
    let mut len_diff = Vector3::zeros();
    let mut index = 0;
    for i in 0..3 {
        if constraints_array[i] {
            len_diff[i] = target_diff[index];
            index += 1;
        }
    }
    let mut rot_diff = Vector3::zeros();
    for i in 0..3 {
        if constraints_array[i + 3] {
            rot_diff[i] = target_diff[index];
            index += 1;
        }
    }
    (len_diff, rot_diff)
}

impl<T> InverseKinematicsSolver<T> for JacobianIKSolver<T>
where
    T: RealField + SubsetOf<f64>,
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
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), Error> {
        self.solve_with_constraints(arm, target_pose, &Constraints::default())
    }

    /// Set joint positions of `arm` to reach the `target_pose` with constraints
    ///
    /// If you want to loose the constraints, use this method.
    /// For example, ignoring rotation with an axis.
    /// It enables to use the arms which has less than six DoF.
    ///
    /// # Example
    ///
    /// ```
    /// use k::prelude::*;
    ///
    /// let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    /// let target_joint_name = "r_wrist_pitch";
    /// let r_wrist = chain.find(target_joint_name).unwrap();
    /// let mut arm = k::SerialChain::from_end(r_wrist);
    /// let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    /// arm.set_joint_positions(&positions).unwrap();
    /// let mut target = arm.update_transforms().last().unwrap().clone();
    /// target.translation.vector.x -= 0.1;
    /// let solver = k::JacobianIKSolver::default();
    ///
    /// let mut constraints = k::Constraints::default();
    /// constraints.rotation_x = false;
    /// constraints.rotation_z = false;
    /// solver
    ///    .solve_with_constraints(&arm, &target, &constraints)
    ///    .unwrap_or_else(|err| {
    ///        println!("Err: {}", err);
    ///    });
    /// ```
    fn solve_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        let orig_positions = arm.joint_positions();
        let re = self.solve_with_constraints_internal(arm, target_pose, constraints);
        if re.is_err() {
            arm.set_joint_positions(&orig_positions)?;
        };
        re
    }
}

impl<T> Default for JacobianIKSolver<T>
where
    T: RealField + SubsetOf<f64>,
{
    fn default() -> Self {
        Self::new(na::convert(0.001), na::convert(0.005), na::convert(0.5), 10)
    }
}

/// Utility function to create nullspace function using reference joint positions.
/// This is just an example to use nullspace.
///
/// H(q) = 1/2(q-q^)T W (q-q^)
/// dH(q) / dq = W (q-q^)
///
/// https://minus9d.hatenablog.com/entry/20120912/1347460308
pub fn create_reference_positions_nullspace_function<T: RealField>(
    reference_positions: Vec<T>,
    weight_vector: Vec<T>,
) -> impl Fn(&[T]) -> Vec<T> {
    let dof = reference_positions.len();
    assert_eq!(dof, weight_vector.len());

    move |positions| {
        let mut derivative_vec = vec![na::convert(0.0); dof];
        for i in 0..dof {
            derivative_vec[i] = weight_vector[i] * (positions[i] - reference_positions[i]);
        }
        derivative_vec
    }
}

#[test]
fn test_nullspace_func() {
    let f = create_reference_positions_nullspace_function(vec![0.0, 1.0], vec![0.5, 0.1]);
    let pos1 = vec![0.5, 0.5];
    let values = f(&pos1);
    assert_eq!(values.len(), 2);
    assert_eq!(values[0], 0.25);
    assert_eq!(values[1], -0.05);
}
