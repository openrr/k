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
use k::urdf::FromUrdf;

fn main() {
    let robot = k::LinkTree::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    let mut arm = k::Manipulator::from_link_tree("l_wrist2", &robot).unwrap();
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
    println!("solved target pos = {}", arm.end_transform().translation);
}
