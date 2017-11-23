// rustup run nightly cargo bench
#![feature(test)]

extern crate test;
extern crate k;
extern crate nalgebra as na;

use k::KinematicChain;
use k::urdf::FromUrdf;
use k::InverseKinematicsSolver;
use k::ChainContainer;

fn bench_tree_ik<K>(arm: &mut K, b: &mut test::Bencher)
where
    K: KinematicChain<f64>,
{
    // set joint angles
    let angles = vec![0.5, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_angles(&angles).unwrap();
    let mut target = arm.calc_end_transform();
    target.translation.vector[0] += 0.02;

    let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 1000);
    b.iter(|| {
        solver.solve(arm, &target).unwrap();
        let _trans = arm.calc_end_transform();
        arm.set_joint_angles(&angles).unwrap();
    });
}


#[bench]
fn bench_rctree_ik(b: &mut test::Bencher) {
    let robot = k::LinkTree::<f64>::from_urdf_file::<f64, _>("urdf/sample.urdf").unwrap();
    let mut arm = robot.get_chain("l_wrist2").unwrap();
    bench_tree_ik(&mut arm, b);
}
