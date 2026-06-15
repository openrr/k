use k::connect;
use k::prelude::*;
use na::{ComplexField, Translation3, Vector3};
use nalgebra as na;
#[cfg(target_family = "wasm")]
use wasm_bindgen_test::wasm_bindgen_test as test;

#[cfg(target_family = "wasm")]
wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

/// Load the redundant 7-DOF fixture from an embedded URDF string (wasm-safe: no file I/O).
fn arm7<T>() -> k::SerialChain<T>
where
    T: na::RealField + k::SubsetOf<f64>,
{
    let robot = urdf_rs::read_from_string(include_str!("../urdf/test_arm7.urdf")).unwrap();
    let chain = k::Chain::<T>::from(&robot);
    let end = chain.find("joint7").unwrap();
    k::SerialChain::from_end(end)
}

fn create_joint_with_link_array6() -> k::SerialChain<f64> {
    let l0: k::Node<f64> = k::NodeBuilder::new()
        .name("shoulder_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .finalize()
        .into();
    let l1: k::Node<f64> = k::NodeBuilder::new()
        .name("shoulder_roll")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f64> = k::NodeBuilder::new()
        .name("shoulder_yaw")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize()
        .into();
    let l3: k::Node<f64> = k::NodeBuilder::new()
        .name("elbow_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l4: k::Node<f64> = k::NodeBuilder::new()
        .name("wrist_yaw")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l5: k::Node<f64> = k::NodeBuilder::new()
        .name("wrist_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    connect![l0 => l1 => l2 => l3 => l4 => l5];
    k::SerialChain::from_end(&l5)
}

fn create_joint_with_link_array7() -> k::SerialChain<f32> {
    let l0: k::Node<f32> = k::NodeBuilder::new()
        .name("shoulder_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .finalize()
        .into();
    let l1: k::Node<f32> = k::NodeBuilder::new()
        .name("shoulder_roll")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f32> = k::NodeBuilder::new()
        .name("shoulder_yaw")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize()
        .into();
    let l3: k::Node<f32> = k::NodeBuilder::new()
        .name("elbow_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l4: k::Node<f32> = k::NodeBuilder::new()
        .name("wrist_yaw")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l5: k::Node<f32> = k::NodeBuilder::new()
        .name("wrist_pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l6: k::Node<f32> = k::NodeBuilder::new()
        .name("wrist_roll")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.10))
        .finalize()
        .into();
    connect![l0 => l1 => l2 => l3 => l4 => l5 => l6];
    k::SerialChain::new_unchecked(k::Chain::from_root(l0))
}

#[test]
fn ik_fk7() {
    let arm = create_joint_with_link_array7();
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
    arm.set_joint_positions(&angles).unwrap();
    let poses = arm.update_transforms();
    let init_pose = poses.last().unwrap();
    let solver = k::JacobianIkSolver::new(0.001, 0.001, 0.5, 100);
    solver.solve(&arm, init_pose).unwrap();
    let end_angles = arm.joint_positions();
    for (init, end) in angles.iter().zip(end_angles.iter()) {
        assert!((init - end).abs() < 0.001);
    }
}

#[test]
fn ik_fk6() {
    let arm = create_joint_with_link_array6();
    let angles = vec![0.8, 0.2, 0.0, -1.2, 0.0, 0.1];
    arm.set_joint_positions(&angles).unwrap();
    let poses = arm.update_transforms();
    let init_pose = poses.last().unwrap();
    let solver = k::JacobianIkSolver::new(0.001, 0.001, 0.8, 100);
    // set different angles
    arm.set_joint_positions(&[0.4, 0.1, 0.1, -1.0, 0.1, 0.1])
        .unwrap();
    solver.solve(&arm, init_pose).unwrap();
    let end_angles = arm.joint_positions();
    println!("{end_angles:?}");
    for (init, end) in angles.iter().zip(end_angles.iter()) {
        assert!((init - end).abs() < 0.002);
    }
}

#[test]
fn ik_fk7_with_constraints() {
    let arm = create_joint_with_link_array7();
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
    arm.set_joint_positions(&angles).unwrap();
    let poses = arm.update_transforms();
    let init_pose = poses.last().unwrap();
    let solver = k::JacobianIkSolver::new(0.001, 0.001, 0.5, 100);
    let constraints = k::Constraints {
        rotation_x: false,
        ignored_joint_names: vec!["wrist_roll".to_string()],
        ..Default::default()
    };
    solver
        .solve_with_constraints(&arm, init_pose, &constraints)
        .unwrap();
    let end_angles = arm.joint_positions();
    for (init, end) in angles.iter().zip(end_angles.iter()) {
        assert!((init - end).abs() < 0.001);
        assert!((angles[6] - end_angles[6]).abs() < f32::EPSILON);
    }
}

// The damped solver is deterministic: the same problem solved twice gives identical positions.
#[test]
fn ik_determinism() {
    let arm = arm7::<f64>();
    arm.set_joint_positions(&[0.3, 0.5, 0.3, 0.4, 0.3, 0.5, 0.3])
        .unwrap();
    let target = arm.end_transform();
    let start = vec![0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.1];
    let solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 50);

    arm.set_joint_positions(&start).unwrap();
    let r1 = solver.solve(&arm, &target);
    let p1 = arm.joint_positions();
    arm.set_joint_positions(&start).unwrap();
    let r2 = solver.solve(&arm, &target);
    let p2 = arm.joint_positions();

    assert_eq!(r1.is_ok(), r2.is_ok());
    assert_eq!(p1, p2);
}

// Provable singularity: a planar two-link arm fully extended along x. Both z-axis joint columns
// point along y, so the 2-by-2 position Jacobian is rank 1; a target further along x is in the
// unreachable/singular direction. The damped solver must stay finite and fail gracefully.
#[test]
fn ik_planar_two_link_singular() {
    let j0: k::Node<f64> = k::NodeBuilder::new()
        .name("j0")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .finalize()
        .into();
    let j1: k::Node<f64> = k::NodeBuilder::new()
        .name("j1")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.5, 0.0, 0.0))
        .finalize()
        .into();
    let tip: k::Node<f64> = k::NodeBuilder::new()
        .name("tip")
        .translation(Translation3::new(0.5, 0.0, 0.0))
        .finalize()
        .into();
    connect![j0 => j1 => tip];
    let arm = k::SerialChain::from_end(&tip);
    arm.set_joint_positions(&[0.0, 0.0]).unwrap();

    // Push the target further along the straight line (+x), into the singular direction.
    let mut target = arm.end_transform();
    target.translation.vector.x += 0.3;
    // Position x,y only => m = 2 = n (square, rank-deficient at this pose).
    let constraints = k::Constraints {
        position_z: false,
        rotation_x: false,
        rotation_y: false,
        rotation_z: false,
        ..Default::default()
    };
    let solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 10);
    let result = solver.solve_with_constraints(&arm, &target, &constraints);

    assert!(arm.joint_positions().iter().all(|x| x.is_finite()));
    assert!(
        result.is_ok() || matches!(result, Err(k::Error::NotConvergedError { .. })),
        "unexpected result: {result:?}"
    );
}

// Near-singular redundant arm: the straight (all-zero) pose with a target pushed beyond reach.
// The undamped solver would blow up; the damped one must stay finite and either converge or
// return NotConvergedError (never panic, no non-finite values). Checked for both f32 and f64.
fn near_singular_arm7<T>()
where
    T: na::RealField + k::SubsetOf<f64>,
{
    let arm = arm7::<T>();
    arm.set_joint_positions(&vec![na::zero::<T>(); 7]).unwrap();
    let mut target = arm.end_transform();
    target.translation.vector.z -= na::convert(0.3);
    let solver = k::JacobianIkSolver::<T>::default(); // small num_max_try (10)
    let result = solver.solve(&arm, &target);

    assert!(arm.joint_positions().iter().all(|x| x.is_finite()));
    assert!(result.is_ok() || matches!(result, Err(k::Error::NotConvergedError { .. })));
}

#[test]
fn ik_arm7_near_singular_f64() {
    near_singular_arm7::<f64>();
}

#[test]
fn ik_arm7_near_singular_f32() {
    near_singular_arm7::<f32>();
}

// Opt-in joint-limit avoidance. joint4 has the tight range [0, 2.6] (mid 1.3). The target is the
// current pose (task already satisfied) with joint4 started near its lower bound, so gain=0 leaves
// it there while gain>0 drifts it toward mid within the null space, away from the bound.
#[test]
fn ik_joint_limit_avoidance() {
    const JOINT4: usize = 3; // tight limit [0.0, 2.6], range center 1.3
    let target_angles = vec![0.5, 0.7, 0.4, 0.2, 0.4, 0.6, 0.3];
    let start = vec![0.2, 0.3, 0.2, 0.2, 0.2, 0.3, 0.2];

    // Relax two orientation axes so the redundant null space (here 3-DOF) actually contains joint4
    // motion. Under full 6-DOF constraints this arm's 1-DOF null space barely involves joint4, so the
    // avoidance — which can only act inside the null space — has nothing to push along there.
    let constraints = k::Constraints {
        rotation_x: false,
        rotation_z: false,
        ..Default::default()
    };

    let arm = arm7::<f64>();
    arm.set_joint_positions(&target_angles).unwrap();
    let target = arm.end_transform();

    // Baseline: gain = 0.
    let baseline = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 100);
    arm.set_joint_positions(&start).unwrap();
    baseline
        .solve_with_constraints(&arm, &target, &constraints)
        .unwrap();
    let joint4_baseline = arm.joint_positions()[JOINT4];

    // Avoidance: gain > 0 pushes joint4 toward the range center 1.3, away from the lower bound 0.
    let mut avoid = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 100);
    avoid.set_joint_limit_avoidance_gain(0.5);
    arm.set_joint_positions(&start).unwrap();
    avoid
        .solve_with_constraints(&arm, &target, &constraints)
        .unwrap();
    let positions = arm.joint_positions();
    let joint4_avoid = positions[JOINT4];

    // (a) every joint within its URDF limits.
    for (i, p) in positions.iter().enumerate() {
        let (lo, hi) = if i == JOINT4 { (0.0, 2.6) } else { (-2.6, 2.6) };
        assert!(
            (lo..=hi).contains(p),
            "joint{} = {p} out of [{lo}, {hi}]",
            i + 1
        );
    }
    // (b) joint4 ends measurably farther from its lower bound than the gain=0 baseline (sign check).
    assert!(
        joint4_avoid > joint4_baseline + 0.02,
        "avoid {joint4_avoid} not farther from lower bound than baseline {joint4_baseline}"
    );
    // (c) the constrained target is still reached.
    let pos_err = (target.translation.vector - arm.end_transform().translation.vector).norm();
    assert!(pos_err < 0.01, "position error too large: {pos_err}");
}

// Redundant constrained solve with an ignored joint on the limited fixture: it converges, every
// joint stays within its URDF limits, and the ignored joint is untouched.
#[test]
fn ik_constraints_ignored_joint_limits() {
    let arm = arm7::<f64>();
    // joint7 (ignored) is held at the same value used to build the target, so it stays reachable.
    arm.set_joint_positions(&[0.3, 0.5, 0.3, 0.4, 0.3, 0.5, 0.7])
        .unwrap();
    let target = arm.end_transform();

    arm.set_joint_positions(&[0.2, 0.4, 0.2, 0.3, 0.2, 0.4, 0.7])
        .unwrap();
    let joint7_initial = arm.joint_positions()[6];
    let constraints = k::Constraints {
        rotation_x: false,
        ignored_joint_names: vec!["joint7".to_string()],
        ..Default::default()
    };
    let solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 100);
    solver
        .solve_with_constraints(&arm, &target, &constraints)
        .unwrap();

    let positions = arm.joint_positions();
    assert!(
        (positions[6] - joint7_initial).abs() < 1e-9,
        "ignored joint moved"
    );
    for (i, p) in positions.iter().enumerate() {
        let (lo, hi) = if i == 3 { (0.0, 2.6) } else { (-2.6, 2.6) };
        assert!(
            (lo..=hi).contains(p),
            "joint{} = {p} out of [{lo}, {hi}]",
            i + 1
        );
    }
}

// A manually locked joint (Range::new(x, x)) must not break the avoidance gradient (its range is 0,
// the divide-by-zero guard returns 0) nor produce non-finite values. Planar three-joint arm,
// position-only (m=2, n=3 redundant).
#[test]
fn ik_locked_joint_finite() {
    let j0: k::Node<f64> = k::NodeBuilder::new()
        .name("j0")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .limits(Some(k::joint::Range::new(-2.6, 2.6)))
        .finalize()
        .into();
    let j1: k::Node<f64> = k::NodeBuilder::new()
        .name("j1")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.4, 0.0, 0.0))
        .limits(Some(k::joint::Range::new(-2.6, 2.6)))
        .finalize()
        .into();
    // Locked: min == max.
    let j2: k::Node<f64> = k::NodeBuilder::new()
        .name("j2")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.4, 0.0, 0.0))
        .limits(Some(k::joint::Range::new(0.3, 0.3)))
        .finalize()
        .into();
    let tip: k::Node<f64> = k::NodeBuilder::new()
        .name("tip")
        .translation(Translation3::new(0.4, 0.0, 0.0))
        .finalize()
        .into();
    connect![j0 => j1 => j2 => tip];
    let arm = k::SerialChain::from_end(&tip);

    arm.set_joint_positions(&[0.2, 0.3, 0.3]).unwrap();
    let target = arm.end_transform();
    arm.set_joint_positions(&[0.1, 0.5, 0.3]).unwrap();

    let mut solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 50);
    solver.set_joint_limit_avoidance_gain(0.5);
    let constraints = k::Constraints {
        position_z: false,
        rotation_x: false,
        rotation_y: false,
        rotation_z: false,
        ..Default::default()
    };
    let result = solver.solve_with_constraints(&arm, &target, &constraints);

    let positions = arm.joint_positions();
    assert!(positions.iter().all(|x| x.is_finite()));
    // The locked joint stays exactly at its single allowed value.
    assert!((positions[2] - 0.3).abs() < 1e-9);
    assert!(result.is_ok() || matches!(result, Err(k::Error::NotConvergedError { .. })));
}

// Zero task error: target equals the current pose, so the solve converges immediately and leaves
// the pose (and joints) unchanged within tolerance.
#[test]
fn ik_zero_error_converges() {
    let arm = arm7::<f64>();
    let angles = vec![0.3, 0.5, 0.3, 0.4, 0.3, 0.5, 0.3];
    arm.set_joint_positions(&angles).unwrap();
    let target = arm.end_transform();
    let solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 10);
    solver.solve(&arm, &target).unwrap();

    let end_err = (target.translation.vector - arm.end_transform().translation.vector).norm();
    assert!(end_err < 1e-6);
    for (a, b) in angles.iter().zip(arm.joint_positions().iter()) {
        assert!((a - b).abs() < 1e-6);
    }
}

// Seeded fuzz: many random reachable targets must never panic or produce non-finite values; each solve
// converges or fails gracefully. (rand is a non-wasm dev-dependency.)
#[cfg(not(target_family = "wasm"))]
#[test]
fn ik_fuzz_arm7_graceful() {
    use rand::rngs::StdRng;
    use rand::{RngExt, SeedableRng};

    fn random_config(rng: &mut StdRng) -> Vec<f64> {
        (0..7)
            .map(|i| {
                let (lo, hi) = if i == 3 { (0.0, 2.6) } else { (-2.6, 2.6) };
                rng.random_range(lo..hi)
            })
            .collect()
    }

    let arm = arm7::<f64>();
    let mut rng = StdRng::seed_from_u64(20_240_615);
    let solver = k::JacobianIkSolver::new(0.001, 0.005, 0.5, 50);
    for _ in 0..200 {
        arm.set_joint_positions(&random_config(&mut rng)).unwrap();
        let target = arm.end_transform();
        arm.set_joint_positions(&random_config(&mut rng)).unwrap();
        let result = solver.solve(&arm, &target);
        assert!(arm.joint_positions().iter().all(|x| x.is_finite()));
        assert!(
            result.is_ok()
                || matches!(
                    result,
                    Err(k::Error::NotConvergedError { .. }) | Err(k::Error::InverseMatrixError)
                ),
            "unexpected result: {result:?}"
        );
    }
}
