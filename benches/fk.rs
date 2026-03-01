use criterion::{criterion_group, criterion_main, Criterion};
use k::joint::Range;
use na::RealField;
use nalgebra as na;
use std::{f64::consts::PI, hint::black_box, sync::Arc, thread};

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

## v0.25.0 on MacBook Pro (Intel Core i7-9750H)

Note that bench_rctree_concurrent_*_N benchmarks N get/set operations from 4 threads.

bench_rctree            time:   [1.6606 us 1.6751 us 1.6907 us]
Found 3 outliers among 100 measurements (3.00%)
  3 (3.00%) high severe

bench_rctree_get_joints time:   [260.14 ns 262.78 ns 266.16 ns]
Found 11 outliers among 100 measurements (11.00%)
  7 (7.00%) high mild
  4 (4.00%) high severe

bench_rctree_set_joints time:   [233.88 ns 234.96 ns 236.25 ns]
Found 5 outliers among 100 measurements (5.00%)
  4 (4.00%) high mild
  1 (1.00%) high severe

bench_rctree_concurrent_set_joints_1000
                        time:   [2.7615 ms 2.7667 ms 2.7727 ms]
Found 5 outliers among 100 measurements (5.00%)
  1 (1.00%) low mild
  4 (4.00%) high severe

bench_rctree_concurrent_get_joints_1000
                        time:   [2.7493 ms 2.7566 ms 2.7651 ms]
Found 5 outliers among 100 measurements (5.00%)
  3 (3.00%) high mild
  2 (2.00%) high severe

bench_rctree_concurrent_set_get_joints_1000
                        time:   [2.7501 ms 2.7562 ms 2.7632 ms]
Found 8 outliers among 100 measurements (8.00%)
  4 (4.00%) high mild
  4 (4.00%) high severe

bench_rctree_ik         time:   [7.2494 us 7.2975 us 7.3501 us]
Found 9 outliers among 100 measurements (9.00%)
  6 (6.00%) high mild
  3 (3.00%) high severe

*/

const THREADS: usize = 4;

fn generate_random_joint_angles_from_limits<T>(limits: &[Option<k::joint::Range<T>>]) -> Vec<T>
where
    T: RealField,
{
    limits
        .iter()
        .map(|range| match range {
            Some(range) => {
                (range.max.clone() - range.min.clone()) * na::convert(rand::random::<f32>())
                    + range.min.clone()
            }
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

fn bench_rctree_get_joints(c: &mut Criterion) {
    let chain = k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap();
    c.bench_function("bench_rctree_get_joints", |b| {
        b.iter(|| {
            black_box(chain.joint_positions());
        });
    });
}

fn bench_rctree_concurrent_set_joints(c: &mut Criterion) {
    let chain = Arc::new(k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap());
    let limits = chain
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();
    let angles = Arc::new(generate_random_joint_angles_from_limits(&limits));
    c.bench_function("bench_rctree_concurrent_set_joints_1000", |b| {
        b.iter(|| {
            let mut handles = vec![];
            for _ in 0..THREADS {
                let chain = chain.clone();
                let angles = angles.clone();
                handles.push(thread::spawn(move || {
                    for _ in 0..1000 {
                        chain.set_joint_positions(&angles).unwrap();
                    }
                }));
            }
            for handle in handles {
                handle.join().unwrap();
            }
        });
    });
}

fn bench_rctree_concurrent_get_joints(c: &mut Criterion) {
    let chain = Arc::new(k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap());
    c.bench_function("bench_rctree_concurrent_get_joints_1000", |b| {
        b.iter(|| {
            let mut handles = vec![];
            for _ in 0..THREADS {
                let chain = chain.clone();
                handles.push(thread::spawn(move || {
                    for _ in 0..1000 {
                        black_box(chain.joint_positions());
                    }
                }));
            }
            for handle in handles {
                handle.join().unwrap();
            }
        });
    });
}

fn bench_rctree_concurrent_set_get_joints(c: &mut Criterion) {
    let chain = Arc::new(k::Chain::<f64>::from_urdf_file("urdf/sample.urdf").unwrap());
    let limits = chain
        .iter_joints()
        .map(|j| j.limits)
        .collect::<Vec<Option<Range<f64>>>>();
    let angles = Arc::new(generate_random_joint_angles_from_limits(&limits));
    c.bench_function("bench_rctree_concurrent_set_get_joints_1000", |b| {
        b.iter(|| {
            let mut handles = vec![];
            for _ in 0..THREADS / 2 {
                let chain = chain.clone();
                handles.push(thread::spawn(move || {
                    for _ in 0..1000 {
                        black_box(chain.joint_positions());
                    }
                }));
            }
            for _ in 0..THREADS / 2 {
                let chain = chain.clone();
                let angles = angles.clone();
                handles.push(thread::spawn(move || {
                    for _ in 0..1000 {
                        chain.set_joint_positions(&angles).unwrap();
                    }
                }));
            }
            for handle in handles {
                handle.join().unwrap();
            }
        });
    });
}

criterion_group!(
    benches,
    bench_rctree,
    bench_rctree_get_joints,
    bench_rctree_set_joints,
    bench_rctree_concurrent_set_joints,
    bench_rctree_concurrent_get_joints,
    bench_rctree_concurrent_set_get_joints,
);
criterion_main!(benches);
