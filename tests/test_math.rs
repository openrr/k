extern crate alga;
extern crate nalgebra as na;
extern crate k;

#[cfg(test)]
mod test {
    use super::*;
    use alga::general::Real;
    #[test]
    pub fn test_to_euler_angles() {
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let rpy = k::math::to_euler_angles(&q);
        assert!((rpy[0] - 0.1).abs() < 0.0001);
        assert!((rpy[1] - 0.2).abs() < 0.0001);
        assert!((rpy[2] - 0.3).abs() < 0.0001);
    }

    #[test]
    pub fn try_pseudo_inverse_identity() {
        let m = na::Matrix3::<f64>::identity();
        assert_eq!(k::math::try_pseudo_inverse(&m).unwrap(), m);

        let m = na::Matrix6::<f32>::identity();
        assert_eq!(k::math::try_pseudo_inverse(&m).unwrap(), m);
    }

    #[test]
    pub fn try_pseudo_inverse() {
        use super::*;
        let m = na::Matrix3::new(1.0, 2.0, 3.0, 4.0, 2.0, 6.0, 7.0, 1.0, 1.0);
        let pseudo = k::math::try_pseudo_inverse(&m).unwrap();
        let pseudo_pseudo = k::math::try_pseudo_inverse(&pseudo).unwrap();
        let small = pseudo_pseudo - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
        let small = m * pseudo * m - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
    }

    #[test]
    #[cfg_attr(rustfmt, rustfmt_skip)]
    pub fn try_pseudo_inverse_65fail() {
        use super::*;
        let m = na::Matrix6x5::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
                                   1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
                                   1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
                                   1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(k::math::try_pseudo_inverse(&m).is_none());
    }

    #[test]
    #[cfg_attr(rustfmt, rustfmt_skip)]
    pub fn try_pseudo_inverse_56() {
        use super::*;
        let m = na::Matrix5x6::new(1.0, 1.0, 1.0, 1.0, 1.0,
                                   2.0, 2.0, 2.0, 2.0, 2.0,
                                   3.0, 3.0, 3.0, 3.0, 3.0,
                                   4.0, 4.0, 4.0, 4.0, 4.0,
                                   5.0, 5.0, 5.0, 5.0, 5.0,
                                   6.0, 6.0, 6.0, 6.0, 6.0);
        let pseudo = k::math::try_pseudo_inverse(&m).unwrap();
        let small = m * pseudo * m - m;
        for val in small.iter() {
            assert!(val.abs() < 0.00001);
        }
    }

}
