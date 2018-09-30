extern crate k;
extern crate nalgebra as na;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    pub fn test_tree() {
        let tree = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
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
}
