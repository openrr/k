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
