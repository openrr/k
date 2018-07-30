extern crate k;
extern crate nalgebra as na;

#[cfg(test)]
mod test {
    use super::*;
    use na::Real;
    #[test]
    pub fn test_to_euler_angles() {
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let rpy = k::math::to_euler_angles(&q);
        assert!((rpy[0] - 0.1).abs() < 0.0001);
        assert!((rpy[1] - 0.2).abs() < 0.0001);
        assert!((rpy[2] - 0.3).abs() < 0.0001);
    }
}
