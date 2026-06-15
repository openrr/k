use criterion::{criterion_group, criterion_main, Criterion};
use k::prelude::*;

// Nominal, well-conditioned pose for the 6-DOF `l_wrist_pitch` arm of `sample.urdf`.
// The bent elbow keeps the Jacobian away from singularities, so the solve converges and
// the timing is a clean speed-regression guard (and stays apples-to-apples across solvers).
const NOMINAL_ANGLES: [f64; 6] = [0.5, 0.2, 0.0, -0.5, 0.0, -0.3];

fn sample_arm() -> k::SerialChain<f64> {
    let robot = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let target_node = robot.find("l_wrist_pitch").unwrap();
    k::SerialChain::from_end(target_node)
}

// Target is the forward-kinematics pose at `NOMINAL_ANGLES` nudged by a small reachable
// translation, so the chosen inputs converge on the pre-DLS solver too.
fn nominal_target(arm: &k::SerialChain<f64>) -> k::Isometry3<f64> {
    arm.set_joint_positions(&NOMINAL_ANGLES).unwrap();
    let mut target = arm.end_transform();
    target.translation.vector[0] += 0.02;
    target
}

// Full 6-DOF constraint: square Jacobian (`m = n = 6`), the original speed-regression guard.
fn bench_wellconditioned(c: &mut Criterion) {
    let arm = sample_arm();
    let target = nominal_target(&arm);
    arm.set_joint_positions(&NOMINAL_ANGLES).unwrap();
    let solver = k::JacobianIkSolver::new(0.001, 0.01, 0.8, 10);
    c.bench_function("bench_wellconditioned", |b| {
        b.iter(|| {
            solver.solve(&arm, &target).unwrap();
            arm.set_joint_positions(&NOMINAL_ANGLES).unwrap();
        });
    });
}

// Relaxing `rotation_x` drops the operational space to `m = 5` while `n = 6`, so the
// Jacobian is wide and the solve takes the redundant null-space path.
fn bench_redundant_constrained(c: &mut Criterion) {
    let arm = sample_arm();
    let target = nominal_target(&arm);
    arm.set_joint_positions(&NOMINAL_ANGLES).unwrap();
    let constraints = k::Constraints {
        rotation_x: false,
        ..Default::default()
    };
    let solver = k::JacobianIkSolver::new(0.001, 0.01, 0.8, 10);
    c.bench_function("bench_redundant_constrained", |b| {
        b.iter(|| {
            solver
                .solve_with_constraints(&arm, &target, &constraints)
                .unwrap();
            arm.set_joint_positions(&NOMINAL_ANGLES).unwrap();
        });
    });
}

criterion_group!(benches, bench_wellconditioned, bench_redundant_constrained);
criterion_main!(benches);
