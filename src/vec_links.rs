use na::{Isometry3, Real};

use links::*;
use joints::*;
use traits::*;
use errors::*;

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
where
    T: Real,
{
    pub fn new(name: &str, frames: Vec<VecKinematicChain<T>>) -> LinkStar<T> {
        LinkStar {
            name: name.to_string(),
            frames: frames,
            transform: Isometry3::identity(),
        }
    }
    pub fn set_transform(&mut self, transform: Isometry3<T>) {
        self.transform = transform;
    }
    pub fn get_transform(&self) -> Isometry3<T> {
        self.transform
    }
}

impl<T> LinkContainer<T> for LinkStar<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        let transforms = self.frames
            .iter()
            .map(|lf| {
                lf.calc_link_transforms()
                    .iter()
                    .map(|&tf| self.transform * tf)
                    .collect()
            })
            .collect::<Vec<Vec<_>>>();
        let mut ret = Vec::new();
        for mut v in transforms {
            ret.append(&mut v);
        }
        ret
    }
    fn get_link_names(&self) -> Vec<String> {
        let names = self.frames
            .iter()
            .map(|lf| lf.get_link_names())
            .collect::<Vec<_>>();
        let mut ret = Vec::new();
        for mut v in names {
            ret.append(&mut v);
        }
        ret
    }
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
where
    T: Real,
{
    pub fn new(name: &str, joint_with_links: Vec<Link<T>>) -> VecKinematicChain<T> {
        VecKinematicChain {
            name: name.to_string(),
            joint_with_links: joint_with_links,
            transform: Isometry3::identity(),
        }
    }
    pub fn len(&self) -> usize {
        self.joint_with_links.len()
    }
    pub fn is_empty(&self) -> bool {
        self.joint_with_links.is_empty()
    }
}

impl<T> LinkContainer<T> for VecKinematicChain<T>
where
    T: Real,
{
    /// returns transforms of links
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.joint_with_links
            .iter()
            .scan(self.transform, |base, lj| {
                *base *= lj.calc_transform();
                Some(*base)
            })
            .collect()
    }
    fn get_link_names(&self) -> Vec<String> {
        self.joint_with_links
            .iter()
            .map(|lj| lj.name.to_owned())
            .collect()
    }
}

impl<T> KinematicChain<T> for VecKinematicChain<T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.joint_with_links.iter().fold(
            self.transform,
            |trans, lj| {
                trans * lj.calc_transform()
            },
        )
    }
}

impl<T> JointContainer<T> for VecKinematicChain<T>
where
    T: Real,
{
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
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.joint_with_links
            .iter()
            .map(|joint_with_link| joint_with_link.joint.limits.clone())
            .collect()
    }
    fn get_joint_names(&self) -> Vec<String> {
        self.joint_with_links
            .iter()
            .map(|joint_with_link| {
                joint_with_link.get_joint_name().to_string()
            })
            .collect()
    }
}
