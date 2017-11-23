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

use k::InverseKinematicsSolver;
use k::JointContainer;
use k::KinematicChain;
use k::ChainContainer;
use k::urdf::FromUrdf;

fn main() {
    let robot = k::LinkTree::from_urdf_file::<f32, _>("urdf/sample.urdf").unwrap();
    let mut arm = robot.get_chain("l_wrist2").unwrap();
    // set joint angles
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3];
    arm.set_joint_angles(&angles).unwrap();
    println!("initial angles={:?}", arm.get_joint_angles());
    // get the transform of the end of the manipulator (forward kinematics)
    let mut target = arm.calc_end_transform();
    println!("initial target pos = {}", target.translation);
    println!("move z: +0.2");
    target.translation.vector[2] += 0.2;
    let solver = k::JacobianIKSolverBuilder::new().finalize();
    // solve and move the manipulator angles
    solver.solve(&mut arm, &target).unwrap_or_else(|err| {
        println!("Err: {}", err);
        0.0f32
    });
    println!("solved angles={:?}", arm.get_joint_angles());
    println!(
        "solved target pos = {}",
        arm.calc_end_transform().translation
    );
}
