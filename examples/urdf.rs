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
extern crate k;

use k::prelude::*;

fn main() {
    // Load urdf file
    let robot = k::LinkTree::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    println!("robot: {}", robot);

    // Set initial joint angles
    let angles = vec![
        0.1, 0.2, 0.0, -0.5, 0.0, -0.3, 0.1, 0.2, 0.0, -0.5, 0.0, -0.3,
    ];

    robot.set_joint_angles(&angles).unwrap();
    println!("initial angles={:?}", robot.joint_angles());

    let target_link_name = "l_wrist2";
    let target_link = robot.find_link("l_wrist2").unwrap();

    // Get the transform of the end of the manipulator (forward kinematics)
    robot.update_transforms();
    let mut target = target_link.world_transform().unwrap();

    println!("initial target pos = {}", target.translation);
    println!("move x: -0.1");
    target.translation.vector.x -= 0.1;

    // Create IK solver with default settings
    let solver = k::JacobianIKSolverBuilder::new().finalize();

    // solve and move the manipulator angles
    solver.solve(&robot, target_link_name, &target).unwrap();
    println!("solved angles={:?}", robot.joint_angles());

    // robot.update_transforms();
    let solved_pose = target_link.world_transform().unwrap();
    println!("solved target pos = {}", solved_pose.translation);
}
