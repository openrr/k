extern crate nalgebra as na;
extern crate k;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    pub fn test_tree() {
        let tree = k::urdf::create_tree_from_file::<f32, _>("urdf/sample.urdf").unwrap();
        assert_eq!(tree.dof(), 12);
        let all_names = tree.get_all_joint_names();
        assert!(all_names.len() == 13);
        println!("{}", all_names[0]);
        assert!(all_names[0] == "root");
        assert!(all_names[1] == "l_shoulder_yaw");

        let names = tree.get_joint_names();
        assert!(names.len() == 12);
        println!("{}", names[0]);
        assert!(names[0] == "l_shoulder_yaw");
    }
}
