// rustup run nightly cargo bench
#![feature(test)]

extern crate k;
extern crate nalgebra as na;
extern crate test;

use k::prelude::*;

fn bench_tree_ik(arm: &k::LinkTree<f64>, target_link: &str, b: &mut test::Bencher) {
    // set joint angles
    let angles = vec![
        0.5, 0.2, 0.0, -0.5, 0.0, -0.3, 0.5, 0.2, 0.0, -0.5, 0.0, -0.3,
    ];
    arm.set_joint_angles(&angles).unwrap();
    arm.update_transforms();
    let mut target = arm
        .find_link(target_link)
        .unwrap()
        .world_transform()
        .unwrap();
    target.translation.vector[0] += 0.02;

    let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 1000);
    b.iter(|| {
        solver.solve(arm, target_link, &target).unwrap();
        arm.set_joint_angles(&angles).unwrap();
    });
}

#[bench]
fn bench_rctree_ik(b: &mut test::Bencher) {
    let robot = k::LinkTree::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    bench_tree_ik(&robot, "l_wrist2", b);
}
