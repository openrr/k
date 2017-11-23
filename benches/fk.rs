// rustup run nightly cargo bench
#![feature(test)]

extern crate k;
extern crate nalgebra as na;
extern crate rand;
extern crate test;

use std::f64::consts::PI;
use k::JointContainer;
use k::LinkContainer;
use k::urdf::FromUrdf;
use na::Real;


fn generate_random_joint_angles_from_limits<T>(limits: &Vec<Option<k::Range<T>>>) -> Vec<T>
where
    T: Real,
{
    limits
        .iter()
        .map(|range| match *range {
            Some(ref range) => (range.max - range.min) * na::convert(rand::random()) + range.min,
            None => na::convert::<f64, T>(rand::random::<f64>() - 0.5) * na::convert(2.0 * PI),
        })
        .collect()
}


#[bench]
fn bench_rctree(b: &mut test::Bencher) {
    let mut robot = k::LinkTree::<f64>::from_urdf_file::<f64, _>("urdf/sample.urdf").unwrap();
    let limits = robot.get_joint_limits();
    let angles = generate_random_joint_angles_from_limits(&limits);
    b.iter(|| {
        robot.set_joint_angles(&angles).unwrap();
        let _trans = robot.calc_link_transforms();
        assert_eq!(_trans.len(), 13);
    });
}

#[bench]
fn bench_rctree_set_joints(b: &mut test::Bencher) {
    let mut robot = k::LinkTree::<f64>::from_urdf_file::<f64, _>("urdf/sample.urdf").unwrap();
    let limits = robot.get_joint_limits();
    let angles = generate_random_joint_angles_from_limits(&limits);
    b.iter(|| { robot.set_joint_angles(&angles).unwrap(); });
}
