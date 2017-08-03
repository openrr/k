extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Unit, UnitQuaternion, Translation3};
use std::error::Error;
use std::fmt;

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

#[derive(Debug, Clone)]
pub enum JointError {
    OutOfLimit,
    SizeMisMatch,
}

impl fmt::Display for JointError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            JointError::OutOfLimit => write!(f, "limit over"),
            JointError::SizeMisMatch => write!(f, "size is invalid"),
        }
    }
}

impl Error for JointError {
    fn description(&self) -> &str {
        match *self {
            JointError::OutOfLimit => "limit over",
            JointError::SizeMisMatch => "size is invalid",
        }
    }
}



/// Robot representation with set of `VecKinematicChain`s
///
/// This contains multiple `VecKinematicChain`.
/// The frames must be serial without branch.
/// root is the only link which has branch.
#[derive(Debug, Clone)]
pub struct LinkStar<T: Real> {
    pub name: String,
    pub frames: Vec<VecKinematicChain<T>>,
    transform: Isometry3<T>,
}

impl<T> LinkStar<T>
    where T: Real
{
    pub fn new(name: &str, frames: Vec<VecKinematicChain<T>>) -> LinkStar<T> {
        LinkStar {
            name: name.to_string(),
            frames: frames,
            transform: Isometry3::identity(),
        }
    }
    pub fn calc_link_transforms(&self) -> Vec<Vec<Isometry3<T>>> {
        self.frames
            .iter()
            .map(|lf| {
                     lf.calc_link_transforms()
                         .iter()
                         .map(|&tf| self.transform * tf)
                         .collect()
                 })
            .collect()
    }
    pub fn set_transform(&mut self, transform: Isometry3<T>) {
        self.transform = transform;
    }
    pub fn get_transform(&self) -> Isometry3<T> {
        self.transform
    }
}

pub trait KinematicChain<T>
    where T: Real
{
    fn calc_end_transform(&self) -> Isometry3<T>;
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError>;
    fn get_joint_angles(&self) -> Vec<T>;
}

/// Set of Joint and Link
///
/// imagine below structure
///
/// `[transform] -> joint_with_links([[joint] -> [Link]] -> [[joint] -> [Link]] -> ...)`
///
/// The order must be ordered.
///
/// - start from root link
/// - end with last link
#[derive(Debug, Clone)]
pub struct VecKinematicChain<T: Real> {
    pub name: String,
    pub joint_with_links: Vec<Link<T>>,
    pub transform: Isometry3<T>,
}

impl<T> VecKinematicChain<T>
    where T: Real
{
    pub fn new(name: &str, joint_with_links: Vec<Link<T>>) -> VecKinematicChain<T> {
        VecKinematicChain {
            name: name.to_string(),
            joint_with_links: joint_with_links,
            transform: Isometry3::identity(),
        }
    }
    /// returns transforms of links
    pub fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.joint_with_links
            .iter()
            .scan(self.transform, |base, lj| {
                *base *= lj.calc_transform();
                Some(*base)
            })
            .collect()
    }
    pub fn len(&self) -> usize {
        self.joint_with_links.len()
    }
    pub fn is_empty(&self) -> bool {
        self.joint_with_links.is_empty()
    }
}

impl<T> KinematicChain<T> for VecKinematicChain<T>
    where T: Real
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.joint_with_links
            .iter()
            .fold(self.transform, |trans, lj| trans * lj.calc_transform())
    }

    /// if failed, joints angles are non determined,
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut joints_with_angle = self.joint_with_links
            .iter_mut()
            .filter(|lj| lj.has_joint_angle())
            .collect::<Vec<_>>();
        if joints_with_angle.len() != angles.len() {
            return Err(JointError::SizeMisMatch);
        }
        for (i, lj) in joints_with_angle.iter_mut().enumerate() {
            try!(lj.set_joint_angle(angles[i]));
        }

        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.joint_with_links
            .iter()
            .filter_map(|joint_with_link| joint_with_link.get_joint_angle())
            .collect()
    }
}

/// Joint and Link
///
#[derive(Debug, Clone)]
pub struct Link<T: Real> {
    pub name: String,
    /// joint instance
    pub joint: Joint<T>,
    /// local transfrom of joint
    pub transform: Isometry3<T>,
    /// cache of world transform
    pub world_transform_cache: Option<Isometry3<T>>,
}

impl<T> Link<T>
    where T: Real
{
    /// Construct a Link from name and joint instance
    ///
    /// You can use LinkBuilder<T> if you want.
    pub fn new(name: &str, joint: Joint<T>) -> Link<T> {
        Link {
            name: name.to_string(),
            joint: joint,
            transform: Isometry3::identity(),
            world_transform_cache: None,
        }
    }
    pub fn get_joint_name(&self) -> &str {
        &self.joint.name
    }
    pub fn calc_transform(&self) -> Isometry3<T> {
        self.transform * self.joint.calc_transform()
    }
    pub fn set_joint_angle(&mut self, angle: T) -> Result<(), JointError> {
        self.joint.set_angle(angle)
    }
    pub fn get_joint_angle(&self) -> Option<T> {
        self.joint.get_angle()
    }
    pub fn has_joint_angle(&self) -> bool {
        match self.joint.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
}

/// min/max range to check the joint position
#[derive(Clone, Debug)]
pub struct Range<T: Real> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
    where T: Real
{
    pub fn is_valid(&self, val: T) -> bool {
        val < self.max && val > self.min
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
    where T: Real
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
                Isometry3::from_parts(Translation3::new(T::zero(), T::zero(), T::zero()),
                                      UnitQuaternion::from_axis_angle(&axis, self.angle))
            }
            JointType::Linear { axis } => {
                Isometry3::from_parts(Translation3::from_vector(&axis.unwrap() * self.angle),
                                      UnitQuaternion::identity())
            }
        }
    }
}


/// Build a `Link<T>`
///
/// # Examples
///
/// ```
/// extern crate nalgebra as na;
/// extern crate k;
/// let l0 = k::LinkBuilder::new()
///     .name("link1")
///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
///     .joint("link_pitch", k::JointType::Rotational{axis: na::Vector3::y_axis()})
///     .finalize();
/// println!("{:?}", l0);
/// ```
#[derive(Debug, Clone)]
pub struct LinkBuilder<T: Real> {
    name: String,
    joint: Joint<T>,
    transform: Isometry3<T>,
}

impl<T> LinkBuilder<T>
    where T: Real
{
    pub fn new() -> LinkBuilder<T> {
        LinkBuilder {
            name: "".to_string(),
            joint: Joint::new("", JointType::Fixed),
            transform: Isometry3::identity(),
        }
    }
    pub fn name(mut self, name: &str) -> LinkBuilder<T> {
        self.name = name.to_string();
        self
    }
    pub fn joint(mut self, name: &str, joint_type: JointType<T>) -> LinkBuilder<T> {
        self.joint = Joint::new(name, joint_type);
        self
    }
    pub fn transform(mut self, transform: Isometry3<T>) -> LinkBuilder<T> {
        self.transform = transform;
        self
    }
    pub fn translation(mut self, translation: Translation3<T>) -> LinkBuilder<T> {
        self.transform.translation = translation;
        self
    }
    pub fn rotation(mut self, rotation: UnitQuaternion<T>) -> LinkBuilder<T> {
        self.transform.rotation = rotation;
        self
    }
    pub fn finalize(self) -> Link<T> {
        Link {
            name: self.name,
            joint: self.joint,
            transform: self.transform,
            world_transform_cache: None,
        }
    }
}
