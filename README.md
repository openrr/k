# `k`: Kinematics library for rust-lang

[![Build Status](https://img.shields.io/github/actions/workflow/status/openrr/k/ci.yml?branch=main&logo=github)](https://github.com/openrr/k/actions) [![crates.io](https://img.shields.io/crates/v/k.svg?logo=rust)](https://crates.io/crates/k) [![codecov](https://codecov.io/gh/openrr/k/branch/main/graph/badge.svg?token=A0MGJ1V6US)](https://codecov.io/gh/openrr/k) [![docs](https://docs.rs/k/badge.svg)](https://docs.rs/k) [![discord](https://dcbadge.vercel.app/api/server/8DAFFKc88B?style=flat)](https://discord.gg/8DAFFKc88B)

`k` has below functionalities.

1. Forward kinematics
1. Inverse kinematics
1. URDF Loader

`k` uses [nalgebra](https://nalgebra.org) as math library.

See [Document](https://docs.rs/k) and examples/ for more details.

## Requirements to build examples

```bash
sudo apt install g++ cmake xorg-dev libglu1-mesa-dev
```

## IK example with GUI

```bash
cargo run --release --example interactive_ik
```

![ik_sample](https://github.com/openrr/k/raw/main/img/screenshot.png)

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
use k::prelude::*;

fn main() {
    // Load urdf file
    let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    println!("chain: {chain}");

    // Set initial joint angles
    let angles = vec![0.2, 0.2, 0.0, -1.0, 0.0, 0.0, 0.2, 0.2, 0.0, -1.0, 0.0, 0.0];

    chain.set_joint_positions(&angles).unwrap();
    println!("initial angles={:?}", chain.joint_positions());

    let target_link = chain.find("l_wrist_pitch").unwrap();

    // Get the transform of the end of the manipulator (forward kinematics)
    chain.update_transforms();
    let mut target = target_link.world_transform().unwrap();

    println!("initial target pos = {}", target.translation);
    println!("move z: +0.1");
    target.translation.vector.z += 0.1;

    // Create IK solver with default settings
    let solver = k::JacobianIkSolver::default();

    // Create a set of joints from end joint
    let arm = k::SerialChain::from_end(target_link);
    // solve and move the manipulator angles
    solver.solve(&arm, &target).unwrap();
    println!("solved angles={:?}", chain.joint_positions());

    chain.update_transforms();
    let solved_pose = target_link.world_transform().unwrap();
    println!("solved target pos = {}", solved_pose.translation);
}
```

## Structure of API

Top level interface is `Chain` struct. It contains `Node`s and they have the relations between nodes (parent/children).
Actual data (joint angle(position), transform between nodes) is stored in `Joint` object inside nodes.

![ik_sample](https://github.com/openrr/k/raw/main/img/chain.png)

You can get local/world transform of nodes. See below figure to understand what is the node's `local_transform()` and `world_transform()`.

![ik_sample](https://github.com/openrr/k/raw/main/img/transform.png)

## `OpenRR` Community

[Here](https://discord.gg/8DAFFKc88B) is a discord server for `OpenRR` users and developers.
