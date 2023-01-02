use k::connect;
use k::prelude::*;
use na::{Translation3, Vector3};
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
