#[cfg(target_family = "wasm")]
use wasm_bindgen_test::wasm_bindgen_test as test;

#[cfg(target_family = "wasm")]
wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

#[test]
fn test_tree() {
    let tree = k::Chain::<f32>::from(
        urdf_rs::read_from_string(include_str!("../urdf/sample.urdf")).unwrap(),
    );
    assert_eq!(tree.dof(), 12);
    let all_names = tree
        .iter()
        .map(|link| link.joint().name.clone())
        .collect::<Vec<_>>();
    assert_eq!(all_names.len(), 13);
    assert_eq!(
        all_names,
        [
            "root",
            "l_shoulder_yaw",
            "l_shoulder_pitch",
            "l_shoulder_roll",
            "l_elbow_pitch",
            "l_wrist_yaw",
            "l_wrist_pitch",
            "r_shoulder_yaw",
            "r_shoulder_pitch",
            "r_shoulder_roll",
            "r_elbow_pitch",
            "r_wrist_yaw",
            "r_wrist_pitch"
        ]
    );

    let names = tree
        .iter_joints()
        .map(|j| j.name.clone())
        .collect::<Vec<_>>();
    assert_eq!(names.len(), 12);
    assert_eq!(names[0], "l_shoulder_yaw");
}

#[allow(clippy::redundant_clone)] // This is intentional.
#[test]
fn test_clone() {
    let tree = k::Chain::<f32>::from(
        urdf_rs::read_from_string(include_str!("../urdf/sample.urdf")).unwrap(),
    );
    let tree2 = tree.clone();
    assert_eq!(tree.dof(), 12);
    assert_eq!(tree2.dof(), 12);
    let pos = vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, -0.1, -0.2];
    let pos0 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    tree.set_joint_positions(&pos).unwrap();
    assert_eq!(tree.joint_positions(), pos);
    assert_eq!(tree2.joint_positions(), pos0);
    tree2.set_joint_positions(&pos).unwrap();
    assert_eq!(tree2.joint_positions(), pos);
    assert_eq!(
        tree.iter().map(|n| n.world_transform()).collect::<Vec<_>>(),
        tree2
            .iter()
            .map(|n| n.world_transform())
            .collect::<Vec<_>>()
    );
}

#[test]
fn test_link_transforms_use_own_joint_frame() {
    let urdf = r#"<?xml version="1.0"?>
<robot name="links">
  <link name="base_link">
    <visual><origin xyz="0 0 0.05"/><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
  <joint name="joint_0" type="revolute">
    <parent link="base_link"/><child link="link_1"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <link name="link_1">
    <inertial><origin xyz="0.1 0 0"/><mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><origin xyz="0.1 0 0"/><geometry><cylinder radius="0.05" length="0.1"/></geometry></visual>
    <collision><origin xyz="0.1 0 0"/><geometry><cylinder radius="0.05" length="0.1"/></geometry></collision>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="link_1"/><child link="link_2"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <link name="link_2">
    <visual><origin xyz="0.1 0 0"/><geometry><sphere radius="0.05"/></geometry></visual>
  </link>
</robot>"#;
    let chain = k::Chain::<f64>::from(urdf_rs::read_from_string(urdf).unwrap());
    // rotating joint_0 must move the visuals of link_1 and link_2, not of base_link
    chain
        .set_joint_positions(&[std::f64::consts::FRAC_PI_2, 0.0])
        .unwrap();
    chain.update_link_transforms();

    let check = |link_name: &str, expected: [f64; 3]| {
        let node = chain
            .iter()
            .find(|n| n.link().as_ref().map(|l| l.name.as_str()) == Some(link_name))
            .unwrap();
        let link = node.link();
        let link = link.as_ref().unwrap();
        let t = link.visuals[0]
            .world_transform()
            .as_ref()
            .unwrap()
            .translation
            .vector;
        assert!(
            (t - k::nalgebra::Vector3::from(expected)).norm() < 1e-9,
            "{link_name} visual at {t:?}, expected {expected:?}"
        );
        (
            link.inertial.world_transform().unwrap(),
            link.collisions
                .first()
                .map(|c| c.world_transform().unwrap()),
        )
    };
    check("base_link", [0.0, 0.0, 0.05]);
    let (inertial, collision) = check("link_1", [0.0, 0.1, 0.2]);
    let expected = k::nalgebra::Vector3::new(0.0, 0.1, 0.2);
    assert!((inertial.translation.vector - expected).norm() < 1e-9);
    assert!((collision.unwrap().translation.vector - expected).norm() < 1e-9);
    check("link_2", [0.0, 0.1, 0.5]);
}
