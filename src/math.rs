extern crate nalgebra as na;


use alga::general::Real;
use na::{Vector3, UnitQuaternion, Dim, Matrix};
use na::allocator::Allocator;

use na::storage::Storage;

/// create rpy angles from quaternion
pub fn to_euler_angles<T: Real>(q: &UnitQuaternion<T>) -> Vector3<T> {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];
    let ysqr = y * y;
    let _1: T = T::one();
    let _2: T = na::convert(2.0);

    // roll
    let t0 = _2 * (w * x + y * z);
    let t1 = _1 - _2 * (x * x + ysqr);
    let roll = t0.atan2(t1);

    // pitch
    let t2 = _2 * (w * y - z * x);
    let t2 = if t2 > _1 { _1 } else { t2 };
    let t2 = if t2 < -_1 { -_1 } else { t2 };
    let pitch = t2.asin();

    // yaw
    let t3 = _2 * (w * z + x * y);
    let t4 = _1 - _2 * (ysqr + z * z);
    let yaw = t3.atan2(t4);

    Vector3::new(roll, pitch, yaw)
}

/// Try to solve pseudo inverse
pub fn try_pseudo_inverse<N, R, C, S>
    (matrix: &Matrix<N, R, C, S>)
     -> Option<Matrix<N, C, R, <S::Alloc as Allocator<N, C, R>>::Buffer>>
    where N: Real,
          R: Dim,
          C: Dim,
          S: Storage<N, R, C>,
          S::Alloc: Allocator<N, C, R> + Allocator<N, R, R>
{
    match (matrix * matrix.transpose()).try_inverse() {
        Some(mat) => Some(matrix.transpose() * mat),
        None => None,
    }
}
