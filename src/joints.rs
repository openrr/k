use na::{Isometry3, Vector3, Unit, UnitQuaternion, Translation3, Real};
use errors::*;

/// Type of Joint, `Fixed`, `Rotational`, `Linear` is supported now
#[derive(Copy, Debug, Clone)]
pub enum JointType<T: Real> {
    /// Fixed joitn
    Fixed,
    /// Rotational joint around axis. angle [rad].
    Rotational { axis: Unit<Vector3<T>> },
    /// Linear joint. angle is length
    Linear { axis: Unit<Vector3<T>> },
}

/// min/max range to check the joint position
#[derive(Clone, Debug)]
pub struct Range<T: Real> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
where
    T: Real,
{
    pub fn new(min: T, max: T) -> Self {
        Range { min: min, max: max }
    }
    pub fn is_valid(&self, val: T) -> bool {
        val <= self.max && val >= self.min
    }
}

/// Joint with type
#[derive(Debug, Clone)]
pub struct Joint<T: Real> {
    pub name: String,
    pub joint_type: JointType<T>,
    pub angle: T,
    pub limits: Option<Range<T>>,
}

impl<T> Joint<T>
where
    T: Real,
{
    pub fn new(name: &str, joint_type: JointType<T>) -> Joint<T> {
        Joint {
            name: name.to_string(),
            joint_type: joint_type,
            angle: T::zero(),
            limits: None,
        }
    }
    pub fn set_limits(&mut self, limits: Option<Range<T>>) {
        self.limits = limits;
    }
    pub fn set_angle(&mut self, angle: T) -> Result<(), JointError> {
        if let JointType::Fixed = self.joint_type {
            return Err(JointError::OutOfLimit);
        }
        if let Some(range) = self.limits.clone() {
            if !range.is_valid(angle) {
                return Err(JointError::OutOfLimit);
            }
        }
        self.angle = angle;
        Ok(())
    }
    pub fn get_angle(&self) -> Option<T> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.angle),
        }
    }
    pub fn calc_transform(&self) -> Isometry3<T> {
        match self.joint_type {
            JointType::Fixed => Isometry3::identity(),
            JointType::Rotational { axis } => {
                Isometry3::from_parts(
                    Translation3::new(T::zero(), T::zero(), T::zero()),
                    UnitQuaternion::from_axis_angle(&axis, self.angle),
                )
            }
            JointType::Linear { axis } => {
                Isometry3::from_parts(
                    Translation3::from_vector(axis.unwrap() * self.angle),
                    UnitQuaternion::identity(),
                )
            }
        }
    }
}
