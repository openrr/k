/*
  Copyright 2017 Takashi Ogura

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
use nalgebra as na;

use k::prelude::*;
use k::{connect, JacobianIKSolver, JointType, NodeBuilder};
use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

fn create_joint_with_link_array() -> k::Node<f32> {
    let fixed: k::Node<f32> = NodeBuilder::new()
        .name("fixed")
        .joint_type(JointType::Fixed)
        .translation(Translation3::new(0.0, 0.0, 0.6))
        .finalize()
        .into();
    let l0: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l1: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_roll")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f32> = NodeBuilder::new()
        .name("shoulder_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize()
        .into();
    let l3: k::Node<f32> = NodeBuilder::new()
        .name("elbow_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l4: k::Node<f32> = NodeBuilder::new()
        .name("wrist_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l5: k::Node<f32> = NodeBuilder::new()
        .name("wrist_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l6: k::Node<f32> = NodeBuilder::new()
        .name("wrist_roll")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.10))
        .finalize()
        .into();
    connect![fixed => l0 => l1 => l2 => l3 => l4 => l5 => l6];
    fixed
}

fn create_ground(window: &mut Window) -> Vec<SceneNode> {
    let mut panels = Vec::new();
    let size = 0.5f32;
    for i in 0..5 {
        for j in 0..5 {
            let mut c0 = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                c0.set_color(0.0, 0.0, 0.8);
            } else {
                c0.set_color(0.2, 0.2, 0.4);
            }
            let x_ind = j as f32 - 2.5;
            let y_ind = i as f32 - 2.5;
            let trans = Isometry3::from_parts(
                Translation3::new(size * x_ind, 0.0, size * y_ind),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            c0.set_local_transformation(trans);
            panels.push(c0);
        }
    }
    panels
}

fn create_cubes(window: &mut Window) -> Vec<SceneNode> {
    let mut c_fixed = window.add_cube(0.05, 0.05, 0.05);
    c_fixed.set_color(0.2, 0.2, 0.2);
    let mut c0 = window.add_cube(0.1, 0.1, 0.1);
    c0.set_color(1.0, 0.0, 1.0);
    let mut c1 = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);
    let mut c2 = window.add_cube(0.1, 0.1, 0.1);
    c2.set_color(0.0, 1.0, 0.0);
    let mut c3 = window.add_cube(0.1, 0.1, 0.1);
    c3.set_color(0.0, 0.5, 1.0);
    let mut c4 = window.add_cube(0.1, 0.1, 0.1);
    c4.set_color(1.0, 0.5, 1.0);
    let mut c5 = window.add_cube(0.1, 0.1, 0.1);
    c5.set_color(0.5, 0.0, 1.0);
    let mut c6 = window.add_cube(0.1, 0.1, 0.1);
    c6.set_color(0.0, 0.5, 0.2);
    vec![c_fixed, c0, c1, c2, c3, c4, c5, c6]
}

fn main() {
    let root = create_joint_with_link_array();
    let arm = k::SerialChain::new_unchecked(k::Chain::from_root(root));

    let mut window = Window::new("k ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = create_cubes(&mut window);
    let angles = vec![0.2, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
    arm.set_joint_positions(&angles).unwrap();
    let base_rot = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, -0.6),
        UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
    );
    arm.iter().next().unwrap().set_origin(
        base_rot
            * Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.6),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
    );
    arm.update_transforms();
    let end = arm.find("wrist_roll").unwrap();
    let mut target = end.world_transform().unwrap().clone();
    let mut c_t = window.add_sphere(0.05);
    c_t.set_color(1.0, 0.2, 0.2);
    let eye = Point3::new(0.5f32, 1.0, 2.0);
    let at = Point3::new(0.0f32, 0.0, 0.0);
    let mut arc_ball = ArcBall::new(eye, at);
    let mut solver = JacobianIKSolver::default();
    //solver.set_nullspace_function(Box::new(|vec| vec.iter().map(|x| -x).collect()));
    let _ = create_ground(&mut window);

    while window.render_with_camera(&mut arc_ball) {
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(code, Action::Release, _) => {
                    match code {
                        Key::Z => {
                            // reset
                            arm.set_joint_positions(&angles).unwrap();
                            arm.update_transforms();
                            target = end.world_transform().unwrap().clone();
                        }
                        Key::F => target.translation.vector[2] += 0.1,
                        Key::B => target.translation.vector[2] -= 0.1,
                        Key::R => target.translation.vector[0] -= 0.1,
                        Key::L => target.translation.vector[0] += 0.1,
                        Key::P => target.translation.vector[1] += 0.1,
                        Key::N => target.translation.vector[1] -= 0.1,
                        Key::X => solver.set_nullspace_function(Box::new(
                            k::create_reference_positions_nullspace_function(
                                vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                vec![0.1, 0.1, 0.1, 1.0, 0.1, 0.5, 0.0],
                            ),
                        )),
                        Key::C => solver.clear_nullspace_function(),
                        Key::J => println!("joint positions: {:?}", arm.joint_positions()),
                        _ => {}
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                _ => {}
            }
        }
        let mut constraints = k::Constraints::default();
        constraints.rotation_x = false;
        solver
            .solve_with_constraints(&arm, &target, &constraints)
            .unwrap_or_else(|err| {
                println!("Err: {}", err);
            });
        c_t.set_local_transformation(target.clone());
        for (i, trans) in arm.update_transforms().iter().enumerate() {
            cubes[i].set_local_transformation(trans.clone());
        }
    }
}
