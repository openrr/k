extern crate k;

use k::InverseKinematicsSolver;
use k::KinematicChain;

fn main() {
    let robot = k::urdf::create_tree_from_file("urdf/sample.urdf").unwrap();
    let mut arms = k::create_kinematic_chains(&robot);
    // set joint angles
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3];
    arms[0].set_joint_angles(&angles).unwrap();
    println!("initial angles={:?}", arms[0].get_joint_angles());
    // get the transform of the end of the manipulator (forward kinematics)
    let mut target = arms[0].calc_end_transform();
    println!("initial target pos = {}", target.translation);
    println!("move z: +0.2");
    target.translation.vector[2] += 0.2;
    let solver = k::JacobianIKSolverBuilder::new().finalize();
    // solve and move the manipulator angles
    solver
        .solve(&mut arms[0], &target)
        .unwrap_or_else(|err| {
                            println!("Err: {}", err);
                            0.0f32
                        });
    println!("solved angles={:?}", arms[0].get_joint_angles());
    println!("solved target pos = {}",
             arms[0].calc_end_transform().translation);
}
