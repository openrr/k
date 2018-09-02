extern crate k;
extern crate nalgebra as na;

#[cfg(test)]
mod tests {
    use super::*;
    use k::prelude::*;
    use na::{Translation3, Vector3};
    pub fn create_joint_with_link_array6() -> (k::LinkTree<f64>, k::LinkNode<f64>) {
        let l0: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint(
                "shoulder_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .finalize()
            .into();
        let l1: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint(
                "shoulder_roll",
                k::JointType::Rotational {
                    axis: Vector3::x_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize()
            .into();
        let l2: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint(
                "shoulder_yaw",
                k::JointType::Rotational {
                    axis: Vector3::z_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize()
            .into();
        let l3: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint(
                "elbow_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        let l4: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint(
                "wrist_yaw",
                k::JointType::Rotational {
                    axis: Vector3::z_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        let l5: k::LinkNode<f64> = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint(
                "wrist_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        l1.set_parent(&l0);
        l2.set_parent(&l1);
        l3.set_parent(&l2);
        l4.set_parent(&l3);
        l5.set_parent(&l4);
        (k::LinkTree::from_end("arm6", l5.clone()), l5)
    }

    pub fn create_joint_with_link_array7() -> (k::LinkTree<f32>, k::LinkNode<f32>) {
        let l0: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint(
                "shoulder_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .finalize()
            .into();
        let l1: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint(
                "shoulder_roll",
                k::JointType::Rotational {
                    axis: Vector3::x_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize()
            .into();
        let l2: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint(
                "shoulder_yaw",
                k::JointType::Rotational {
                    axis: Vector3::z_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize()
            .into();
        let l3: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint(
                "elbow_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        let l4: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint(
                "wrist_yaw",
                k::JointType::Rotational {
                    axis: Vector3::z_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        let l5: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint(
                "wrist_pitch",
                k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize()
            .into();
        let l6: k::LinkNode<f32> = k::LinkBuilder::new()
            .name("wrist_link3")
            .joint(
                "wrist_roll",
                k::JointType::Rotational {
                    axis: Vector3::x_axis(),
                },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.10))
            .finalize()
            .into();
        l1.set_parent(&l0);
        l2.set_parent(&l1);
        l3.set_parent(&l2);
        l4.set_parent(&l3);
        l5.set_parent(&l4);
        l6.set_parent(&l5);
        (k::LinkTree::from_root("arm", l0), l6)
    }

    #[test]
    pub fn ik_fk7() {
        let (mut arm, end) = create_joint_with_link_array7();
        let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
        arm.set_joint_angles(&angles).unwrap();
        let poses = arm.update_transforms();
        let init_pose = poses.last().unwrap();
        let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 100);
        solver
            .solve(&mut arm, &end.link_name(), &init_pose)
            .unwrap();
        let end_angles = arm.joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }

    #[test]
    pub fn ik_fk6() {
        let (mut arm, end) = create_joint_with_link_array6();
        let angles = vec![0.8, 0.2, 0.0, -1.2, 0.0, 0.1];
        arm.set_joint_angles(&angles).unwrap();
        let poses = arm.update_transforms();
        let init_pose = poses.last().unwrap();
        let solver = k::JacobianIKSolverBuilder::new().finalize();
        // set different angles
        arm.set_joint_angles(&[0.4, 0.1, 0.1, -1.0, 0.1, 0.1])
            .unwrap();
        solver
            .solve(&mut arm, &end.link_name(), &init_pose)
            .unwrap();
        let end_angles = arm.joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }
}
