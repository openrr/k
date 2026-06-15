//! Robustness / accuracy benchmark harness for `JacobianIkSolver`.
//!
//! Sweeps a fixed, seeded set of IK problems in three regimes — well-conditioned,
//! near-singular, and redundant-with-limits — and prints a markdown table of success
//! rate, panic / non-finite counts, and final residual. It calls only the preserved
//! public API and `urdf/sample.urdf`, so the same source can be run against the solver
//! before and after the Damped-Least-Squares upgrade to compare robustness.
//!
//! Run with `cargo run --release --example ik_benchmark`.

#[cfg(not(target_family = "wasm"))]
fn main() {
    imp::run();
}

// `rand` is only a dev-dependency off-wasm and CI does not build examples for wasm, so the
// harness compiles to an empty entry point there.
#[cfg(target_family = "wasm")]
fn main() {}

#[cfg(not(target_family = "wasm"))]
mod imp {
    use k::prelude::*;
    use k::{Chain, Constraints, Isometry3, JacobianIkSolver, SerialChain};
    use rand::rngs::StdRng;
    use rand::{RngExt, SeedableRng};
    use std::panic::{catch_unwind, AssertUnwindSafe};

    // Fixed seed => identical target sweep on every run, so `before.md`/`after.md` diff cleanly.
    const SEED: u64 = 20_240_614;
    // Number of sampled problems per regime.
    const N_PER_SET: usize = 200;
    // Well-conditioned reference pose for the 6-DOF arm (bent elbow, away from singularities).
    const NOMINAL: [f64; 6] = [0.3, 0.2, 0.0, -0.8, 0.2, 0.1];

    struct Case {
        start: Vec<f64>,
        target: Isometry3<f64>,
        constraints: Constraints,
    }

    #[derive(Default)]
    struct SetResult {
        successes: usize,
        panics: usize,
        non_finite: usize,
        sum_pos: f64,
        max_pos: f64,
        sum_rot: f64,
        max_rot: f64,
    }

    impl SetResult {
        fn record_success(&mut self, pos: f64, rot: f64) {
            self.successes += 1;
            self.sum_pos += pos;
            self.sum_rot += rot;
            self.max_pos = self.max_pos.max(pos);
            self.max_rot = self.max_rot.max(rot);
        }
    }

    fn arm() -> SerialChain<f64> {
        let robot = Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
        let end = robot.find("l_wrist_pitch").unwrap();
        SerialChain::from_end(end)
    }

    // Forward kinematics: end pose of `a` at `angles`.
    fn fk(a: &SerialChain<f64>, angles: &[f64]) -> Isometry3<f64> {
        a.set_joint_positions(angles).unwrap();
        a.end_transform()
    }

    // Well-conditioned: random reachable targets a small perturbation away from `NOMINAL`.
    fn wellconditioned_cases(a: &SerialChain<f64>, rng: &mut StdRng) -> Vec<Case> {
        (0..N_PER_SET)
            .map(|_| {
                let mut q = NOMINAL;
                for v in &mut q {
                    *v += rng.random_range(-0.2..0.2);
                }
                Case {
                    start: NOMINAL.to_vec(),
                    target: fk(a, &q),
                    constraints: Constraints::default(),
                }
            })
            .collect()
    }

    // Near-singular: start almost straight (elbow/wrist aligned) and push the target along the
    // shoulder->end reach line to/just past the workspace boundary, where the undamped Jacobian
    // loses rank. `extra < 0` is reachable, `extra > 0` is just beyond reach.
    fn near_singular_cases(a: &SerialChain<f64>, rng: &mut StdRng) -> Vec<Case> {
        (0..N_PER_SET)
            .map(|_| {
                let mut start = [0.0f64; 6];
                for v in &mut start {
                    *v += rng.random_range(-0.03..0.03);
                }
                a.set_joint_positions(&start).unwrap();
                a.update_transforms();
                let base = a.iter().next().unwrap().world_transform().unwrap();
                let end = a.end_transform();
                let reach = (end.translation.vector - base.translation.vector).normalize();
                let extra = rng.random_range(-0.03..0.18);
                let mut target = end;
                target.translation.vector += reach * extra;
                Case {
                    start: start.to_vec(),
                    target,
                    constraints: Constraints::default(),
                }
            })
            .collect()
    }

    // Redundant + limits: relax `rotation_x` (m=5 < n=6) so the null-space path runs, with larger
    // perturbations so the reachable targets drive joints toward the URDF limits.
    fn redundant_cases(a: &SerialChain<f64>, rng: &mut StdRng) -> Vec<Case> {
        let constraints = Constraints {
            rotation_x: false,
            ..Default::default()
        };
        (0..N_PER_SET)
            .map(|_| {
                let mut q = NOMINAL;
                for v in &mut q {
                    *v += rng.random_range(-0.6..0.6);
                }
                Case {
                    start: NOMINAL.to_vec(),
                    target: fk(a, &q),
                    constraints: constraints.clone(),
                }
            })
            .collect()
    }

    // Each solve runs on an independent clone of the arm so a panicking solve cannot poison later
    // cases: chain nodes are `Arc<Mutex<..>>`, and a panic mid-solve would poison shared mutexes.
    fn run_set(
        base: &SerialChain<f64>,
        solver: &JacobianIkSolver<f64>,
        cases: &[Case],
    ) -> SetResult {
        let mut res = SetResult::default();
        for case in cases {
            let solved = base.clone();
            let start = case.start.clone();
            let target = case.target;
            let constraints = case.constraints.clone();
            let outcome = catch_unwind(AssertUnwindSafe(move || {
                solved.set_joint_positions(&start).unwrap();
                let ok = solver
                    .solve_with_constraints(&solved, &target, &constraints)
                    .is_ok();
                (ok, solved.joint_positions(), solved.end_transform())
            }));
            match outcome {
                // Solver panicked (e.g. an undamped `.unwrap()` on a singular factorization).
                Err(_) => res.panics += 1,
                Ok((ok, positions, end)) => {
                    if !positions.iter().all(|p| p.is_finite()) {
                        res.non_finite += 1;
                    } else if ok {
                        let pos = (case.target.translation.vector - end.translation.vector).norm();
                        let rot = case.target.rotation.angle_to(&end.rotation);
                        if pos.is_finite() && rot.is_finite() {
                            res.record_success(pos, rot);
                        } else {
                            res.non_finite += 1;
                        }
                    }
                }
            }
        }
        res
    }

    fn print_row(name: &str, r: &SetResult) {
        let succ = r.successes;
        let panics = r.panics;
        let nf = r.non_finite;
        let rate = 100.0 * succ as f64 / N_PER_SET as f64;
        let (mean_pos, max_pos, mean_rot, max_rot) = if succ > 0 {
            (
                format!("{:.2e}", r.sum_pos / succ as f64),
                format!("{:.2e}", r.max_pos),
                format!("{:.2e}", r.sum_rot / succ as f64),
                format!("{:.2e}", r.max_rot),
            )
        } else {
            (
                "—".to_owned(),
                "—".to_owned(),
                "—".to_owned(),
                "—".to_owned(),
            )
        };
        println!(
            "| {name} | {N_PER_SET} | {succ} | {rate:.1}% | {panics} | {nf} | {mean_pos} | {max_pos} | {mean_rot} | {max_rot} |"
        );
    }

    pub(crate) fn run() {
        let base = arm();
        let solver = JacobianIkSolver::default();
        let mut rng = StdRng::seed_from_u64(SEED);

        // Build every set before solving so the RNG draw order (hence the sweep) is independent
        // of solver behavior and bit-identical across runs.
        let well = wellconditioned_cases(&base, &mut rng);
        let near = near_singular_cases(&base, &mut rng);
        let redundant = redundant_cases(&base, &mut rng);

        let well_res = run_set(&base, &solver, &well);
        let near_res = run_set(&base, &solver, &near);
        let redundant_res = run_set(&base, &solver, &redundant);

        println!("# IK solver robustness / accuracy benchmark\n");
        println!("- Arm: `urdf/sample.urdf` `l_wrist_pitch` (6 DOF)");
        println!("- Samples per set: {N_PER_SET}");
        println!("- Seed: `{SEED}`");
        println!("- Residuals (meters / radians) are over successful solves only (`end_transform` vs target).\n");
        println!("| Target set | Samples | Success | Rate | Panics | Non-finite | Pos mean | Pos max | Rot mean | Rot max |");
        println!("|---|---|---|---|---|---|---|---|---|---|");
        print_row("Well-conditioned", &well_res);
        print_row("Near-singular", &near_res);
        print_row("Redundant + limits", &redundant_res);
    }
}
