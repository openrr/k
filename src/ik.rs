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

use std::fmt;

use na::{DVector, Isometry3, RealField, Vector3, Vector6};
use nalgebra as na;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use simba::scalar::SubsetOf;

use super::chain::*;
use super::errors::*;
use super::funcs::*;

/// From 'Humanoid Robot (Kajita)' P.64
fn calc_pose_diff<T>(a: &Isometry3<T>, b: &Isometry3<T>) -> Vector6<T>
where
    T: RealField,
{
    let p_diff = a.translation.vector.clone() - b.translation.vector.clone();
    let w_diff = b.rotation.rotation_to(&a.rotation).scaled_axis();
    Vector6::new(
        p_diff[0].clone(),
        p_diff[1].clone(),
        p_diff[2].clone(),
        w_diff[0].clone(),
        w_diff[1].clone(),
        w_diff[2].clone(),
    )
}

fn calc_pose_diff_with_constraints<T>(
    a: &Isometry3<T>,
    b: &Isometry3<T>,
    operational_space: [bool; 6],
) -> DVector<T>
where
    T: RealField,
{
    let full_diff = calc_pose_diff(a, b);
    let use_dof = operational_space.iter().filter(|x| **x).count();
    let mut diff = DVector::from_element(use_dof, na::zero());
    let mut index = 0;
    for (i, use_i) in operational_space.iter().enumerate() {
        if *use_i {
            diff[index] = full_diff[i].clone();
            index += 1;
        }
    }
    diff
}

/// A bundle of flags determining which coordinates are constrained for a target
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Constraints {
    /// true means the constraint is used.
    ///  The coordinates is the world, not the end of the arm.
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub position_x: bool,
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub position_y: bool,
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub position_z: bool,
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub rotation_x: bool,
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub rotation_y: bool,
    #[cfg_attr(feature = "serde", serde(default = "default_true"))]
    pub rotation_z: bool,
    #[cfg_attr(feature = "serde", serde(default))]
    pub ignored_joint_names: Vec<String>,
}

fn default_true() -> bool {
    true
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
    /// assert!(c.ignored_joint_names.is_empty());
    /// ```
    fn default() -> Self {
        Self {
            position_x: default_true(),
            position_y: default_true(),
            position_z: default_true(),
            rotation_x: default_true(),
            rotation_y: default_true(),
            rotation_z: default_true(),
            ignored_joint_names: Default::default(),
        }
    }
}

fn define_operational_space(constraints: &Constraints) -> [bool; 6] {
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

/// Default manipulability threshold `w0`: dynamic damping turns on once the Yoshikawa
/// manipulability measure `w = sqrt(det(J J^T))` drops below it. Kept small so that
/// well-conditioned configurations get zero damping and the step is the exact minimum-norm
/// pseudoinverse. Scale-dependent — see `set_manipulability_threshold`.
const DEFAULT_MANIPULABILITY_THRESHOLD: f64 = 1e-3;
/// Default maximum squared damping factor `lambda^2_max`, approached as `w -> 0`.
const DEFAULT_MAX_DAMPING_SQUARED: f64 = 1e-3;
/// Default joint-limit-avoidance gain `k0`. Zero disables the opt-in null-space avoidance, so the
/// solver is numerically identical to plain damped least squares unless a caller raises it.
const DEFAULT_JOINT_LIMIT_AVOIDANCE_GAIN: f64 = 0.0;

/// Inverse Kinematics Solver using the Jacobian matrix with Damped Least Squares (DLS).
///
/// Each iteration takes a damped least squares (Levenberg-Marquardt) step
/// `J^T (J J^T + lambda^2 I)^-1 e`. The damping `lambda^2` is dynamic: it is recomputed every
/// iteration from the Yoshikawa manipulability measure `w = sqrt(det(J J^T))`, so it is zero for
/// well-conditioned configurations — where the step reduces exactly to the minimum-norm
/// pseudoinverse — and grows smoothly toward `max_damping_squared` as the arm approaches a
/// singularity, keeping the step bounded and the solve panic-free.
///
/// `manipulability_threshold` (`w0`) is scale-dependent: `w` mixes length-unit translation rows
/// with dimensionless rotation rows and varies with the active constraints, so a good value depends
/// on the robot's size and the task. The default is deliberately small; if it is too small for the
/// robot scale a near-singular step fails cleanly with `InverseMatrixError` rather than blowing up.
pub struct JacobianIkSolver<T: RealField> {
    /// If the distance is smaller than this value, it is reached.
    pub allowable_target_distance: T,
    /// If the angle distance is smaller than this value, it is reached.
    pub allowable_target_angle: T,
    /// multiplier for jacobian
    pub jacobian_multiplier: T,
    /// How many times the joints are tried to be moved
    pub num_max_try: usize,
    /// Manipulability threshold `w0`: damping engages once `w = sqrt(det(J J^T))` drops below it.
    /// Scale-dependent; see the type-level docs and `set_manipulability_threshold`.
    pub manipulability_threshold: T,
    /// Maximum squared damping factor `lambda^2_max`, approached as the manipulability `w -> 0`.
    pub max_damping_squared: T,
    /// Joint-limit-avoidance gain `k0` for the opt-in null-space secondary task (Liegeois 1977).
    /// Defaults to 0 (avoidance off). Only acts on redundant systems (`n > m`); see
    /// `set_joint_limit_avoidance_gain`.
    pub joint_limit_avoidance_gain: T,
    /// Nullspace function for a redundant system
    #[allow(clippy::type_complexity)]
    nullspace_function: Option<Box<dyn Fn(&[T]) -> Vec<T> + Send + Sync>>,
}

/// Dynamic damping factor `lambda^2` for damped least squares, from the Yoshikawa manipulability
/// measure `w` and the threshold `w0`:
///
/// ```text
/// lambda^2 = 0                                 if w >= w0   (or w0 <= 0)
/// lambda^2 = lambda_squared_max * (1 - w/w0)^2 if 0 <= w < w0
/// ```
///
/// Zero for well-conditioned configurations (`w >= w0`), so the damped step reduces exactly to the
/// undamped minimum-norm pseudoinverse; it rises to `lambda_squared_max` as `w -> 0`. The quadratic
/// is continuous at `w0` (both branches give 0 there). Standalone for unit testing.
fn damping_squared<T: RealField>(w: T, w0: T, lambda_squared_max: T) -> T {
    // A non-positive threshold means "never damp"; `w >= w0` is the well-conditioned region.
    if w0 <= T::zero() || w >= w0 {
        return T::zero();
    }
    // 0 <= w < w0  =>  1 - w/w0 in (0, 1];  lambda^2 = lambda_squared_max * (1 - w/w0)^2.
    let factor = T::one() - w / w0;
    lambda_squared_max * factor.clone() * factor
}

/// Liegeois (1977) joint-limit-avoidance gradient, used as the secondary null-space task: each
/// component pushes a joint toward the center of its range, weighted so a joint near a bound moves
/// more strongly.
///
/// `subtask_i = gain * (mid_i - theta_i) / (max_i - min_i)^2`, with `mid_i = (min_i + max_i) / 2`.
/// The sign descends the cost `H = sum_i ((theta_i - mid_i) / (max_i - min_i))^2`, i.e. it moves
/// toward `mid` (avoidance). A joint with no limit (`None`) or a degenerate range (`max <= min`, e.g.
/// a locked joint) contributes `0` — this is the divide-by-zero guard. The result drops the
/// `ignored_joint_indices` components so it lines up with the reduced `n`-column Jacobian.
fn joint_limit_avoidance_gradient<T: RealField>(
    gain: T,
    positions: &[T],
    limits: &[Option<(T, T)>],
    ignored_joint_indices: &[usize],
) -> DVector<T> {
    let mut subtask: Vec<T> = positions
        .iter()
        .zip(limits)
        .map(|(theta, limit)| match limit {
            Some((min, max)) => {
                let range = max.clone() - min.clone();
                if range <= T::zero() {
                    T::zero()
                } else {
                    let mid = (min.clone() + max.clone()) / (T::one() + T::one());
                    gain.clone() * (mid - theta.clone()) / (range.clone() * range)
                }
            }
            None => T::zero(),
        })
        .collect();
    // Indices are sorted ascending; removing in order keeps the remaining components aligned.
    for (i, joint_index) in ignored_joint_indices.iter().enumerate() {
        subtask.remove(*joint_index - i);
    }
    DVector::from_vec(subtask)
}

impl<T> JacobianIkSolver<T>
where
    T: RealField + SubsetOf<f64>,
{
    /// Create instance of `JacobianIkSolver`.
    ///
    /// Dynamic damping starts from the default manipulability threshold and maximum damping; adjust
    /// them afterwards with `set_manipulability_threshold` and `set_max_damping`.
    ///
    /// # Examples
    ///
    /// ```
    /// let solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 100);
    /// ```
    pub fn new(
        allowable_target_distance: T,
        allowable_target_angle: T,
        jacobian_multiplier: T,
        num_max_try: usize,
    ) -> JacobianIkSolver<T> {
        JacobianIkSolver {
            allowable_target_distance,
            allowable_target_angle,
            jacobian_multiplier,
            num_max_try,
            manipulability_threshold: na::convert(DEFAULT_MANIPULABILITY_THRESHOLD),
            max_damping_squared: na::convert(DEFAULT_MAX_DAMPING_SQUARED),
            joint_limit_avoidance_gain: na::convert(DEFAULT_JOINT_LIMIT_AVOIDANCE_GAIN),
            nullspace_function: None,
        }
    }
    /// Set a null space function for redundant manipulator.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 100);
    /// solver.set_nullspace_function(Box::new(
    /// k::create_reference_positions_nullspace_function(
    ///    vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ///    vec![0.1, 0.1, 0.1, 1.0, 0.1, 0.5, 0.0],
    ///    ),
    /// ));
    /// ```
    #[allow(clippy::type_complexity)]
    pub fn set_nullspace_function(&mut self, func: Box<dyn Fn(&[T]) -> Vec<T> + Send + Sync>) {
        self.nullspace_function = Some(func);
    }

    /// Clear the null function which is set by `set_nullspace_function`.
    pub fn clear_nullspace_function(&mut self) {
        self.nullspace_function = None;
    }

    /// Set the manipulability threshold `w0` below which dynamic damping engages.
    ///
    /// `w0` is compared against the Yoshikawa manipulability `w = sqrt(det(J J^T))`. It is
    /// scale-dependent (the Jacobian mixes translation and rotation rows, and the row count changes
    /// with the active constraints), so tune it for the robot: too large over-damps and slows
    /// convergence, too small lets near-singular steps grow until the factorization fails with
    /// `InverseMatrixError`. A non-positive value disables damping entirely.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 100);
    /// solver.set_manipulability_threshold(1e-4);
    /// ```
    pub fn set_manipulability_threshold(&mut self, manipulability_threshold: T) {
        self.manipulability_threshold = manipulability_threshold;
    }

    /// Set the maximum squared damping factor `lambda^2_max` approached as the manipulability
    /// `w -> 0`. Larger values give more stability near singularities at the cost of a larger
    /// first-order perturbation of the primary task.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 100);
    /// solver.set_max_damping(1e-2);
    /// ```
    pub fn set_max_damping(&mut self, max_damping_squared: T) {
        self.max_damping_squared = max_damping_squared;
    }

    /// Set the joint-limit-avoidance gain `k0`, enabling the opt-in null-space secondary task that
    /// pushes redundant joints toward the center of their range (Liegeois 1977). `0` (the default)
    /// disables it, leaving the solve numerically identical to plain damped least squares. The
    /// avoidance only acts on redundant systems (`n > m`) and perturbs the primary task to first
    /// order, so keep `k0` small; a joint may need a larger `num_max_try` to be pulled off a bound.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 100);
    /// solver.set_joint_limit_avoidance_gain(0.1);
    /// ```
    pub fn set_joint_limit_avoidance_gain(&mut self, joint_limit_avoidance_gain: T) {
        self.joint_limit_avoidance_gain = joint_limit_avoidance_gain;
    }

    fn add_positions_with_multiplier(&self, input: &[T], add_values: &[T]) -> Vec<T> {
        input
            .iter()
            .zip(add_values.iter())
            .map(|(ang, add)| ang.clone() + self.jacobian_multiplier.clone() * add.clone())
            .collect()
    }

    fn solve_one_loop_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        operational_space: &[bool; 6],
        ignored_joint_indices: &[usize],
    ) -> Result<DVector<T>, Error> {
        let required_dof = operational_space.iter().filter(|x| **x).count();
        let orig_positions = arm.joint_positions();
        let available_dof = arm.dof() - ignored_joint_indices.len();

        // required_dof == 0 means no operational constraints: nothing to solve, no empty determinant.
        if required_dof == 0 {
            return Ok(DVector::zeros(0));
        }

        // Task error e (length m = required_dof), sign target - end.
        let err =
            calc_pose_diff_with_constraints(target_pose, &arm.end_transform(), *operational_space);

        // Geometric Jacobian reduced to the kept operational rows and the movable columns:
        // J is m x n with m = required_dof, n = available_dof, and n >= m (enforced before the loop).
        let mut jacobi = jacobian(arm);
        let mut num_removed_rows = 0;
        for (i, use_i) in operational_space.iter().enumerate() {
            if !use_i {
                jacobi = jacobi.remove_row(i - num_removed_rows);
                num_removed_rows += 1;
            }
        }
        for (i, joint_index) in ignored_joint_indices.iter().enumerate() {
            jacobi = jacobi.remove_column(*joint_index - i);
        }
        let j_t = jacobi.transpose(); // n x m

        // Damped least squares: invert the m x m Gram matrix J J^T (not the n x n J^T J, which is
        // singular when n > m), keeping the factorization at most 6 by 6 and reusing det(J J^T).
        let mut gram = &jacobi * &j_t; // J J^T (m x m)

        // Yoshikawa manipulability w = sqrt(det(J J^T)); clamp the determinant to >= 0 first
        // (round-off can make a rank-deficient Gram matrix slightly negative before the sqrt).
        let w = gram.determinant().max(T::zero()).sqrt();
        let lambda_squared = damping_squared(
            w,
            self.manipulability_threshold.clone(),
            self.max_damping_squared.clone(),
        );

        // A = J J^T + lambda^2 I. Symmetric positive definite when lambda^2 > 0 or J has full row
        // rank; Cholesky factors it once and returns None on a true rank loss (lambda^2 = 0, singular
        // J), which becomes a graceful InverseMatrixError so the outer loop restores the positions.
        for i in 0..required_dof {
            gram[(i, i)] = gram[(i, i)].clone() + lambda_squared.clone();
        }
        let cholesky = gram.cholesky().ok_or(Error::InverseMatrixError)?;

        // Primary task step  d_q = J^T (J J^T + lambda^2 I)^-1 e.
        let mut d_q = &j_t * cholesky.solve(&err);

        // Optional secondary task projected into the null space of the primary task, without forming
        // the n x n projector:  d_q += (I - J^T A^-1 J) g. Only meaningful when redundant (n > m).
        // g sums the opt-in joint-limit-avoidance gradient (when the gain is non-zero) and the user
        // nullspace gradient, both reduced over the ignored joints. With gain 0 and no nullspace
        // function nothing is added, so the step is identical to plain damped least squares.
        if available_dof > required_dof {
            let mut g: Option<DVector<T>> = None;
            if self.joint_limit_avoidance_gain != T::zero() {
                // Read the limits into owned data; the JointRefGuards are dropped here, before any
                // matrix math or set_* call (holding a guard across set_joint_positions_* deadlocks).
                let limits: Vec<Option<(T, T)>> = arm
                    .iter_joints()
                    .map(|joint| {
                        joint
                            .limits
                            .as_ref()
                            .map(|range| (range.min.clone(), range.max.clone()))
                    })
                    .collect();
                g = Some(joint_limit_avoidance_gradient(
                    self.joint_limit_avoidance_gain.clone(),
                    &orig_positions,
                    &limits,
                    ignored_joint_indices,
                ));
            }
            if let Some(ref f) = self.nullspace_function {
                let mut user = DVector::from_vec(f(&orig_positions));
                for (i, joint_index) in ignored_joint_indices.iter().enumerate() {
                    user = user.remove_row(*joint_index - i);
                }
                g = Some(match g {
                    Some(limit) => limit + user,
                    None => user,
                });
            }
            if let Some(g) = g {
                let z = cholesky.solve(&(&jacobi * &g));
                d_q += g - &j_t * z;
            }
        }

        // Re-insert zero steps for the ignored joints so d_q matches the full joint vector.
        for joint_index in ignored_joint_indices {
            d_q = d_q.insert_row(*joint_index, T::zero());
        }

        // Reject a non-finite step instead of corrupting the chain state.
        if d_q.iter().any(|v| !v.is_finite()) {
            return Err(Error::InverseMatrixError);
        }

        let positions_vec = self.add_positions_with_multiplier(&orig_positions, d_q.as_slice());
        arm.set_joint_positions_clamped(&positions_vec);
        Ok(calc_pose_diff_with_constraints(
            target_pose,
            &arm.end_transform(),
            *operational_space,
        ))
    }

    fn solve_with_constraints_internal(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        let operational_space = define_operational_space(constraints);
        let required_dof = operational_space.iter().filter(|x| **x).count();
        let orig_positions = arm.joint_positions();
        let available_dof = arm.dof() - constraints.ignored_joint_names.len();
        if available_dof < required_dof {
            return Err(Error::PreconditionError {
                dof: available_dof,
                necessary_dof: required_dof,
            });
        }
        let mut ignored_joint_indices = Vec::new();
        for joint_name in &constraints.ignored_joint_names {
            // Try to get joint index
            match arm.iter_joints().position(|x| x.name == *joint_name) {
                Some(index) => {
                    ignored_joint_indices.push(index);
                }
                None => {
                    return Err(Error::InvalidJointNameError {
                        joint_name: joint_name.to_string(),
                    });
                }
            }
        }
        ignored_joint_indices.sort_unstable();
        let mut last_target_distance = None;
        for _ in 0..self.num_max_try {
            let target_diff = self.solve_one_loop_with_constraints(
                arm,
                target_pose,
                &operational_space,
                &ignored_joint_indices,
            )?;
            let (len_diff, rot_diff) = target_diff_to_len_rot_diff(&target_diff, operational_space);
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
            position_diff: na::try_convert(last_target_distance.as_ref().unwrap().0.clone())
                .unwrap_or_default(),
            rotation_diff: na::try_convert(last_target_distance.as_ref().unwrap().1.clone())
                .unwrap_or_default(),
        })
    }
}

impl<T: RealField + fmt::Debug> fmt::Debug for JacobianIkSolver<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JacobianIkSolver")
            .field("allowable_target_distance", &self.allowable_target_distance)
            .field("allowable_target_angle", &self.allowable_target_angle)
            .field("jacobian_multiplier", &self.jacobian_multiplier)
            .field("num_max_try", &self.num_max_try)
            .field("manipulability_threshold", &self.manipulability_threshold)
            .field("max_damping_squared", &self.max_damping_squared)
            .field(
                "joint_limit_avoidance_gain",
                &self.joint_limit_avoidance_gain,
            )
            .field("has_nullspace_function", &self.nullspace_function.is_some())
            .finish()
    }
}

fn target_diff_to_len_rot_diff<T>(
    target_diff: &DVector<T>,
    operational_space: [bool; 6],
) -> (Vector3<T>, Vector3<T>)
where
    T: RealField,
{
    let mut len_diff = Vector3::zeros();
    let mut index = 0;
    for i in 0..3 {
        if operational_space[i] {
            len_diff[i] = target_diff[index].clone();
            index += 1;
        }
    }
    let mut rot_diff = Vector3::zeros();
    for i in 0..3 {
        if operational_space[i + 3] {
            rot_diff[i] = target_diff[index].clone();
            index += 1;
        }
    }
    (len_diff, rot_diff)
}

impl<T> InverseKinematicsSolver<T> for JacobianIkSolver<T>
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
    /// println!("arm: {arm}");
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
    /// let solver = k::JacobianIkSolver::default();
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
    /// let solver = k::JacobianIkSolver::default();
    ///
    /// let mut constraints = k::Constraints::default();
    /// constraints.rotation_x = false;
    /// constraints.rotation_z = false;
    /// solver
    ///    .solve_with_constraints(&arm, &target, &constraints)
    ///    .unwrap_or_else(|err| {
    ///        println!("Err: {err}");
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

impl<T> Default for JacobianIkSolver<T>
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
/// <https://minus9d.hatenablog.com/entry/20120912/1347460308>
pub fn create_reference_positions_nullspace_function<T: RealField>(
    reference_positions: Vec<T>,
    weight_vector: Vec<T>,
) -> impl Fn(&[T]) -> Vec<T> {
    let dof = reference_positions.len();
    assert_eq!(dof, weight_vector.len());

    move |positions| {
        let mut derivative_vec = vec![na::convert(0.0); dof];
        for i in 0..dof {
            derivative_vec[i] =
                weight_vector[i].clone() * (positions[i].clone() - reference_positions[i].clone());
        }
        derivative_vec
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(target_family = "wasm")]
    use wasm_bindgen_test::wasm_bindgen_test as test;

    #[test]
    fn test_nullspace_func() {
        let f = create_reference_positions_nullspace_function(vec![0.0, 1.0], vec![0.5, 0.1]);
        let pos1 = vec![0.5, 0.5];
        let values = f(&pos1);
        assert_eq!(values.len(), 2);
        assert!((values[0] - 0.25f64).abs() < f64::EPSILON);
        assert!((values[1] - (-0.05f64)).abs() < f64::EPSILON);
    }

    #[test]
    fn test_damping_squared() {
        let w0 = 1e-3_f64;
        let max = 1e-3_f64;
        // No damping in the well-conditioned region w >= w0 (continuous: w == w0 also gives 0).
        assert_eq!(damping_squared(2e-3, w0, max), 0.0);
        assert_eq!(damping_squared(w0, w0, max), 0.0);
        // Full damping as w -> 0.
        assert!((damping_squared(0.0, w0, max) - max).abs() < 1e-15);
        // Quadratic midpoint: lambda_squared_max * (1 - 1/2)^2 = lambda_squared_max / 4.
        assert!((damping_squared(w0 / 2.0, w0, max) - max / 4.0).abs() < 1e-15);
        // Monotonically decreasing in w on (0, w0).
        let a = damping_squared(0.2e-3, w0, max);
        let b = damping_squared(0.5e-3, w0, max);
        let c = damping_squared(0.8e-3, w0, max);
        assert!(a > b && b > c);
        // A non-positive threshold disables damping for any w.
        assert_eq!(damping_squared(0.5, 0.0, max), 0.0);
        assert_eq!(damping_squared(0.5, -1.0, max), 0.0);
    }

    #[test]
    fn test_joint_limit_avoidance_gradient() {
        let gain = 1.0_f64;
        // Joints: symmetric [-1,1] (mid 0); locked [2,2]; unlimited; [0,2] (mid 1).
        let limits = vec![Some((-1.0, 1.0)), Some((2.0, 2.0)), None, Some((0.0, 2.0))];
        let positions = vec![0.0, 2.0, 5.0, 1.9];
        let g = joint_limit_avoidance_gradient(gain, &positions, &limits, &[]);
        assert_eq!(g.len(), 4);
        // At mid -> 0.
        assert!((g[0] - 0.0).abs() < 1e-12);
        // Locked (max == min) -> 0 and finite (divide-by-zero guard).
        assert_eq!(g[1], 0.0);
        assert!(g[1].is_finite());
        // No limit -> 0.
        assert_eq!(g[2], 0.0);
        // Near max -> negative: (1 - 1.9) / 2^2 = -0.225.
        assert!(g[3] < 0.0);
        assert!((g[3] - (-0.225)).abs() < 1e-12);
        // Near min -> positive.
        let near_min = joint_limit_avoidance_gradient(gain, &[0.1], &[Some((0.0, 2.0))], &[]);
        assert!(near_min[0] > 0.0);
        // Reduction drops the ignored joint (index 1), keeping the rest aligned.
        let reduced = joint_limit_avoidance_gradient(gain, &positions, &limits, &[1]);
        assert_eq!(reduced.len(), 3);
        assert_eq!(reduced[0], g[0]);
        assert_eq!(reduced[1], g[2]);
        assert_eq!(reduced[2], g[3]);
    }
}
