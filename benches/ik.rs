use criterion::{criterion_group, criterion_main, Bencher, Criterion};
use k::prelude::*;

fn bench_tree_ik(robot: &k::Chain<f64>, target_link: &str, b: &mut Bencher<'_>) {
    let target_node = robot.find(target_link).unwrap();
    let arm = k::SerialChain::from_end(target_node);
    // set joint angles
    let angles = vec![0.5, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&angles).unwrap();
    arm.update_transforms();
    let mut target = target_node.world_transform().unwrap();
    target.translation.vector[0] += 0.02;
    let solver = k::JacobianIkSolver::new(0.001, 0.01, 0.8, 10);
    b.iter(|| {
        solver.solve(&arm, &target).unwrap();
        arm.set_joint_positions(&angles).unwrap();
    });
}

fn bench_rctree_ik(c: &mut Criterion) {
    let robot = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    c.bench_function("bench_rctree_ik", |b| {
        bench_tree_ik(&robot, "l_wrist_pitch", b);
    });
}

criterion_group!(benches, bench_rctree_ik);
criterion_main!(benches);
