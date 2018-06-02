// rustup run nightly cargo bench
#![feature(test)]

extern crate k;
extern crate nalgebra as na;
extern crate test;

use k::urdf::FromUrdf;
use k::{EndTransform, HasJoints, InverseKinematicsSolver};

fn bench_tree_ik<K>(arm: &mut K, b: &mut test::Bencher)
where
    K: HasJoints<f64> + EndTransform<f64>,
{
    // set joint angles
    let angles = vec![0.5, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_angles(&angles).unwrap();
    let mut target = arm.end_transform();
    target.translation.vector[0] += 0.02;

    let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 1000);
    b.iter(|| {
        solver.solve(arm, &target).unwrap();
        let _trans = arm.end_transform();
        arm.set_joint_angles(&angles).unwrap();
    });
}

#[bench]
fn bench_rctree_ik(b: &mut test::Bencher) {
    let robot = k::LinkTree::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let mut arm = k::Manipulator::from_link_tree("l_wrist2", &robot).unwrap();
    bench_tree_ik(&mut arm, b);
}
