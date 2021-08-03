use criterion::{criterion_group, criterion_main, Criterion};
use k::joint::Range;
use na::RealField;
use nalgebra as na;
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

## v0.17.0 on MacBook Pro (Retina, 13-inch, Early 2015, 2.9GHz - 16GB)
test bench_rctree            ... bench:       1,855 ns/iter (+/- 165)
test bench_rctree_set_joints ... bench:         186 ns/iter (+/- 6)
test bench_rctree_ik ... bench:       8,640 ns/iter (+/- 364)

## v0.19.0 on Desktop (AMD Ryzen 9 3950X 16-Core Processor, 3.5GHz - 64GB)

test bench_rctree            ... bench:       1,293 ns/iter (+/- 17)
test bench_rctree_set_joints ... bench:          98 ns/iter (+/- 2)
test bench_rctree_ik ... bench:       4,394 ns/iter (+/- 73)

## v0.20.0(Arc) on Desktop (AMD Ryzen 9 3950X 16-Core Processor, 3.5GHz - 64GB)

test bench_rctree            ... bench:       1,803 ns/iter (+/- 21)
test bench_rctree_set_joints ... bench:         204 ns/iter (+/- 17)
test bench_rctree_ik ... bench:       5,985 ns/iter (+/- 65)
*/

fn generate_random_joint_angles_from_limits<T>(limits: &[Option<k::joint::Range<T>>]) -> Vec<T>
where
    T: RealField,
{
    limits
        .iter()
        .map(|range| match *range {
            Some(ref range) => (range.max - range.min) * na::convert(rand::random()) + range.min,
            None => na::convert::<f64, T>(rand::random::<f64>() - 0.5) * na::convert(2.0 * PI),
        })
        .collect()
}

fn bench_rctree(c: &mut Criterion) {
    let chain = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let limits = chain
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();
    let angles = generate_random_joint_angles_from_limits(&limits);
    c.bench_function("bench_rctree", |b| {
        b.iter(|| {
            chain.set_joint_positions(&angles).unwrap();
            let _trans = chain.update_transforms();
            assert_eq!(_trans.len(), 13);
        });
    });
}

fn bench_rctree_set_joints(c: &mut Criterion) {
    let chain = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    let limits = chain
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();
    let angles = generate_random_joint_angles_from_limits(&limits);
    c.bench_function("bench_rctree_set_joints", |b| {
        b.iter(|| {
            chain.set_joint_positions(&angles).unwrap();
        });
    });
}

criterion_group!(benches, bench_rctree, bench_rctree_set_joints);
criterion_main!(benches);
