#[cfg(target_arch = "wasm32")]
use wasm_bindgen_test::wasm_bindgen_test as test;

#[cfg(target_arch = "wasm32")]
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
    assert!(all_names.len() == 13);
    println!("{}", all_names[0]);
    assert!(all_names[0] == "root");
    assert!(all_names[1] == "r_shoulder_yaw");

    let names = tree
        .iter_joints()
        .map(|j| j.name.clone())
        .collect::<Vec<_>>();
    assert!(names.len() == 12);
    println!("{}", names[0]);
    assert!(names[0] == "r_shoulder_yaw");
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
