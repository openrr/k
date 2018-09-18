// rustup run nightly cargo bench
#![feature(test)]

extern crate k;
extern crate nalgebra as na;
extern crate test;

use k::prelude::*;

fn bench_tree_ik(robot: &k::Chain<f64>, target_link: &str, b: &mut test::Bencher) {
    let target_node = robot.find(target_link).unwrap();
    let arm = k::SerialChain::from_end(&target_node);
    // set joint angles
    let angles = vec![0.5, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&angles).unwrap();
    arm.update_transforms();
    let mut target = target_node.world_transform().unwrap();
    target.translation.vector[0] += 0.02;
    let solver = k::JacobianIKSolver::new(0.001, 0.01, 0.8, 10);
    b.iter(|| {
        solver.solve(&arm, &target).unwrap();
        arm.set_joint_positions(&angles).unwrap();
    });
}

#[bench]
fn bench_rctree_ik(b: &mut test::Bencher) {
    let robot = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    bench_tree_ik(&robot, "l_wrist_pitch", b);
}
