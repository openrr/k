# `k`: Kinematics library for rust-lang [![Build Status](https://travis-ci.org/OTL/k.svg?branch=master)](https://travis-ci.org/OTL/k) [![crates.io](https://img.shields.io/crates/v/k.svg)](https://crates.io/crates/k)

`k` has below functionalities.

1. Forward kinematics
1. Inverse kinematics
1. URDF Loader

`k` uses [nalgebra](http://nalgebra.org) as math library.

See [Document](http://docs.rs/k) and examples/ for more details.

## IK example with GUI

```bash
cargo run --release --example interactive_ik
```

![ik_sample](img/screenshot.png)

Push below keys to move the end of the manipulator.

- `f`: forward
- `b`: backward
- `p`: up
- `n`: down
- `l`: left
- `r`: right
- `z`: reset the manipulator state.

## Create link tree from urdf and solve IK

```rust
extern crate k;

use k::InverseKinematicsSolver;
use k::JointContainer;
use k::Manipulator;
use k::ChainContainer;
use k::urdf::FromUrdf;

fn main() {
    let robot = k::LinkTree::from_urdf_file::<f32, _>("urdf/sample.urdf").unwrap();
    let mut arm = robot.new_chain("l_wrist2").unwrap();
    // set joint angles
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3];
    arm.set_joint_angles(&angles).unwrap();
    println!("initial angles={:?}", arm.joint_angles());
    // get the transform of the end of the manipulator (forward kinematics)
    let mut target = arm.end_transform();
    println!("initial target pos = {}", target.translation);
    println!("move z: +0.2");
    target.translation.vector[2] += 0.2;
    let solver = k::JacobianIKSolverBuilder::new().finalize();
    // solve and move the manipulator angles
    solver.solve(&mut arm, &target).unwrap_or_else(|err| {
        println!("Err: {}", err);
        0.0f32
    });
    println!("solved angles={:?}", arm.joint_angles());
    println!(
        "solved target pos = {}",
        arm.end_transform().translation
    );
}
```
