extern crate nalgebra as na;
extern crate k;

#[cfg(test)]
mod tests {
    use super::*;
    use k::urdf::FromUrdf;
    use k::JointContainer;

    #[test]
    pub fn test_tree() {
        let tree = k::LinkTree::<f32>::from_urdf_file::<f32, _>("urdf/sample.urdf").unwrap();
        assert_eq!(tree.dof(), 12);
        let all_names = tree.iter_link()
            .map(|link| link.get_joint_name().to_string())
            .collect::<Vec<_>>();
        assert!(all_names.len() == 13);
        println!("{}", all_names[0]);
        assert!(all_names[0] == "root");
        assert!(all_names[1] == "r_shoulder_yaw");

        let names = tree.get_joint_names();
        assert!(names.len() == 12);
        println!("{}", names[0]);
        assert!(names[0] == "r_shoulder_yaw");
    }
}
