use approx::assert_abs_diff_eq;
use k::{connect, joint::Range, prelude::*};
use na::{Translation3, UnitQuaternion, Vector3};
use nalgebra as na;
#[cfg(target_family = "wasm")]
use wasm_bindgen_test::wasm_bindgen_test as test;

#[cfg(target_family = "wasm")]
wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

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

#[test]
fn test_ik_with_ignored_joints() {
    let base: k::Node<f64> = k::NodeBuilder::new()
        .name("base")
        .joint_type(k::JointType::Fixed)
        .rotation(UnitQuaternion::from_euler_angles(
            0.0,
            0.0,
            90.0_f64.to_radians(),
        ))
        .finalize()
        .into();

    let linear_z: k::Node<f64> = k::NodeBuilder::new()
        .name("linear_1")
        .joint_type(k::JointType::Linear {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, 1.2))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .finalize()
        .into();

    let shoulder: k::Node<f64> = k::NodeBuilder::new()
        .name("shoulder")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.2, 0.0, 0.1))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .limits(Some(Range::new(
            -90.0_f64.to_radians(),
            90.0_f64.to_radians(),
        )))
        .finalize()
        .into();

    let elbow: k::Node<f64> = k::NodeBuilder::new()
        .name("elbow")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.5, 0.0, -0.1))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 180.0))
        .limits(Some(Range::new(
            -90.0_f64.to_radians(),
            90.0_f64.to_radians(),
        )))
        .finalize()
        .into();

    let yaw: k::Node<f64> = k::NodeBuilder::new()
        .name("yaw")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.5, 0.0, 0.02))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .limits(Some(Range::new(
            -90.0_f64.to_radians(),
            90.0_f64.to_radians(),
        )))
        .finalize()
        .into();

    let linear_y: k::Node<f64> = k::NodeBuilder::new()
        .name("linear_y")
        .joint_type(k::JointType::Linear {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.1, 0.0, -0.1))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .finalize()
        .into();

    let pitch: k::Node<f64> = k::NodeBuilder::new()
        .name("pitch")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.2, 0.0, -0.3))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .limits(Some(Range::new(
            -45.0_f64.to_radians(),
            45.0_f64.to_radians(),
        )))
        .finalize()
        .into();

    let linear_x: k::Node<f64> = k::NodeBuilder::new()
        .name("linear_x")
        .joint_type(k::JointType::Linear {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.1, 0.0, -0.1))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .finalize()
        .into();

    let roll: k::Node<f64> = k::NodeBuilder::new()
        .name("roll")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.1, 0.0, 0.1))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .limits(Some(Range::new(
            -30.0_f64.to_radians(),
            30.0_f64.to_radians(),
        )))
        .finalize()
        .into();

    let tool: k::Node<f64> = k::NodeBuilder::new()
        .name("tool")
        .joint_type(k::JointType::Fixed)
        .translation(Translation3::new(0.1, 0.0, 0.0))
        .rotation(UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0))
        .finalize()
        .into();

    connect![base => linear_z => shoulder => elbow => yaw => linear_y => pitch => linear_x => roll => tool];

    let chain = k::SerialChain::from_end(&roll);

    let _result = chain.set_joint_positions(&[
        0.5,                   // linear_z
        60.0_f64.to_radians(), // shoulder
        30.0_f64.to_radians(), // wrist
        5.0_f64.to_radians(),  // yaw
        0.0,                   // linear_y
        5.0_f64.to_radians(),  // pitch
        0.0,                   // linear_x
        5.0_f64.to_radians(),  // roll
    ]);

    let tool_pose = chain.end_transform();

    let solver = k::JacobianIkSolver::new(1e-6, 1e-6, 0.5, 1000);
    let constraints = k::Constraints {
        ignored_joint_names: vec!["linear_x".to_string(), "linear_y".to_string()],
        ..Default::default()
    };

    chain.set_joint_positions(&[0.0; 8]).unwrap();

    solver
        .solve_with_constraints(&chain, &tool_pose, &constraints)
        .unwrap();

    let ik_tool_pose = chain.end_transform();

    assert_abs_diff_eq!(tool_pose, ik_tool_pose, epsilon = 1e-6);
}
