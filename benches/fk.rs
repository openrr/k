// rustup run nightly cargo bench
#![feature(test)]

extern crate k;
extern crate nalgebra as na;
extern crate rand;
extern crate test;

use na::Real;
use std::f64::consts::PI;

/*
## v0.10.1 on MacBook Pro (Retina, 13-inch, Early 2015, 2.9GHz - 16GB)

```
test bench_rctree            ... bench:       3,000 ns/iter (+/- 31)
test bench_rctree_set_joints ... bench:          92 ns/iter (+/- 1)
test bench_rctree_ik ... bench:      28,555 ns/iter (+/- 263)
```

## v0.11.0 on MacBook Pro (Retina, 13-inch, Early 2015, 2.9GHz - 16GB)

```
test bench_rctree            ... bench:       2,203 ns/iter (+/- 12)
test bench_rctree_set_joints ... bench:          25 ns/iter (+/- 0)
test bench_rctree_ik ... bench:      20,425 ns/iter (+/- 139)
```

## v0.12.1 on MacBook Pro (Retina, 13-inch, Early 2015, 2.9GHz - 16GB)

```
test bench_rctree            ... bench:       3,107 ns/iter (+/- 2,837)
test bench_rctree_set_joints ... bench:         296 ns/iter (+/- 30)
test bench_rctree_ik ... bench:      10,622 ns/iter (+/- 1,203)
```
*/

fn generate_random_joint_angles_from_limits<T>(limits: &Vec<Option<k::joint::Range<T>>>) -> Vec<T>
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
    let chain = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let limits = chain.iter_joints().map(|j| j.limits.clone()).collect();
    let angles = generate_random_joint_angles_from_limits(&limits);
    b.iter(|| {
        chain.set_joint_positions(&angles).unwrap();
        let _trans = chain.update_transforms();
        assert_eq!(_trans.len(), 13);
    });
}

#[bench]
fn bench_rctree_set_joints(b: &mut test::Bencher) {
    let chain = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let limits = chain.iter_joints().map(|j| j.limits.clone()).collect();
    let angles = generate_random_joint_angles_from_limits(&limits);
    b.iter(|| {
        chain.set_joint_positions(&angles).unwrap();
    });
}
