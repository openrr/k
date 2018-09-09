# `k`: Kinematics library for rust-lang [![Build Status](https://travis-ci.org/OTL/k.svg?branch=master)](https://travis-ci.org/OTL/k) [![crates.io](https://img.shields.io/crates/v/k.svg)](https://crates.io/crates/k)

`k` has below functionalities.

1. Forward kinematics
1. Inverse kinematics
1. URDF Loader

`k` uses [nalgebra](http://nalgebra.org) as math library.

See [Document](http://docs.rs/k) and examples/ for more details.

## Requirements to build examples

```bash
sudo apt install g++ cmake xorg-dev libglu1-mesa-dev
```

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

use k::prelude::*;

fn main() {
    // Load urdf file
    let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    println!("chain: {}", chain);

    // Set initial joint angles
    let angles = vec![0.2, 0.2, 0.0, -1.0, 0.0, 0.0, 0.2, 0.2, 0.0, -1.0, 0.0, 0.0];

    chain.set_joint_angles(&angles).unwrap();
    println!("initial angles={:?}", chain.joint_angles());

    let target_link_name = "l_wrist2";
    let target_link = chain.find_link("l_wrist2").unwrap();

    // Get the transform of the end of the manipulator (forward kinematics)
    chain.update_transforms();
    let mut target = target_link.world_transform().unwrap();

    println!("initial target pos = {}", target.translation);
    println!("move z: +0.1");
    target.translation.vector.z += 0.1;

    // Create IK solver with default settings
    let solver = k::JacobianIKSolverBuilder::new().finalize();

    // solve and move the manipulator angles
    solver.solve(&chain, target_link_name, &target).unwrap();
    println!("solved angles={:?}", chain.joint_angles());

    // chain.update_transforms();
    let solved_pose = target_link.world_transform().unwrap();
    println!("solved target pos = {}", solved_pose.translation);
}
```
