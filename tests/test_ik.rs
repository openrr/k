extern crate alga;
extern crate nalgebra as na;
extern crate k;

#[cfg(test)]
mod tests {
    use super::*;
    use na::{Vector3, Translation3};
    use k::InverseKinematicsSolver;
    use k::KinematicChain;

    pub fn create_joint_with_link_array6(name: &str) -> k::VecKinematicChain<f32> {
        let l0 = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint("shoulder_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .finalize();
        let l1 = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint("shoulder_roll",
                   k::JointType::Rotational { axis: Vector3::x_axis() })
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize();
        let l2 = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint("shoulder_yaw",
                   k::JointType::Rotational { axis: Vector3::z_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize();
        let l3 = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint("elbow_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l4 = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint("wrist_yaw",
                   k::JointType::Rotational { axis: Vector3::z_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l5 = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint("wrist_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        k::VecKinematicChain::new(name, vec![l0, l1, l2, l3, l4, l5])
    }


    pub fn create_joint_with_link_array7(name: &str) -> k::VecKinematicChain<f32> {
        let l0 = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint("shoulder_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .finalize();
        let l1 = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint("shoulder_roll",
                   k::JointType::Rotational { axis: Vector3::x_axis() })
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize();
        let l2 = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint("shoulder_yaw",
                   k::JointType::Rotational { axis: Vector3::z_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize();
        let l3 = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint("elbow_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l4 = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint("wrist_yaw",
                   k::JointType::Rotational { axis: Vector3::z_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l5 = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint("wrist_pitch",
                   k::JointType::Rotational { axis: Vector3::y_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l6 = k::LinkBuilder::new()
            .name("wrist_link3")
            .joint("wrist_roll",
                   k::JointType::Rotational { axis: Vector3::x_axis() })
            .translation(Translation3::new(0.0, 0.0, -0.10))
            .finalize();
        k::VecKinematicChain::new(name, vec![l0, l1, l2, l3, l4, l5, l6])
    }

    #[test]
    pub fn ik_fk7() {
        let mut arm = create_joint_with_link_array7("arm7");
        let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
        arm.set_joint_angles(&angles).unwrap();
        let init_pose = arm.calc_end_transform();
        let solver = k::JacobianIKSolver::new(0.001, 0.001, 100);
        solver.solve(&mut arm, &init_pose).unwrap();
        let end_angles = arm.get_joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }

    #[test]
    pub fn ik_fk6() {
        let mut arm = create_joint_with_link_array6("arm6");
        let angles = vec![0.8, 0.2, 0.0, -1.2, 0.0, 0.1];
        arm.set_joint_angles(&angles).unwrap();
        let init_pose = arm.calc_end_transform();
        let solver = k::JacobianIKSolver::new(0.001, 0.001, 100);
        // set different angles
        arm.set_joint_angles(&[0.4, 0.1, 0.1, -1.0, 0.1, 0.1])
            .unwrap();
        solver.solve(&mut arm, &init_pose).unwrap();
        let end_angles = arm.get_joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }
}
