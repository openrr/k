use super::chain::*;
use super::joint::*;
use na::{DMatrix, RealField, Vector3};
use nalgebra as na;
use simba::scalar::SubsetOf;

/// Calculate Jacobian of the serial chain (manipulator).
pub fn jacobian<T>(arm: &SerialChain<T>) -> DMatrix<T>
where
    T: RealField + SubsetOf<f64>,
{
    let dof = arm.dof();
    let t_n = arm.end_transform();
    arm.update_transforms();
    let p_n = t_n.translation;
    let jacobi_vec = arm
        .iter_joints()
        .map(|joint| {
            let t_i = joint.world_transform().unwrap();
            match &joint.joint_type {
                JointType::Linear { axis } => {
                    let p_i = t_i.rotation * axis;
                    [
                        p_i[0].clone(),
                        p_i[1].clone(),
                        p_i[2].clone(),
                        na::zero(),
                        na::zero(),
                        na::zero(),
                    ]
                }
                JointType::Rotational { axis } => {
                    let p_i = t_i.translation;
                    let a_i = t_i.rotation * axis;
                    let dp_i = a_i.cross(&(p_n.clone().vector - p_i.vector));
                    [
                        dp_i[0].clone(),
                        dp_i[1].clone(),
                        dp_i[2].clone(),
                        a_i[0].clone(),
                        a_i[1].clone(),
                        a_i[2].clone(),
                    ]
                }
                JointType::Fixed => panic!("impossible, bug of jacobian"),
            }
        })
        .collect::<Vec<_>>();
    // Pi: a_i x (p_n - Pi)
    // wi: a_i
    DMatrix::from_fn(6, dof, |r, c| jacobi_vec[c][r].clone())
}

/// Calculate the center of mass of the chain
///
/// ```
/// use k::*;
/// use k::link::*;
///
/// let j0 = NodeBuilder::new()
///     .translation(Translation3::new(0.0, 1.0, 0.0))
///     .into_node();
/// let j1 = NodeBuilder::new()
///     .translation(Translation3::new(0.0, 0.0, 1.0))
///     .into_node();
/// j0.set_link(Some(LinkBuilder::new().inertial(Inertial::from_mass(1.0)).finalize()));
/// j1.set_link(Some(LinkBuilder::new().inertial(Inertial::from_mass(4.0)).finalize()));
/// j1.set_parent(&j0);
/// let tree = Chain::from_root(j0);
/// let com1 = center_of_mass(&tree);
/// ```
pub fn center_of_mass<T>(chain: &Chain<T>) -> Vector3<T>
where
    T: RealField + SubsetOf<f64>,
{
    let mut total_mass = T::zero();
    let mut com = Vector3::zeros();

    chain.update_transforms();
    chain.iter().for_each(|node| {
        if let Some(trans) = node.world_transform() {
            if let Some(ref link) = *node.link() {
                let inertia_trans = trans * link.inertial.origin().translation.clone();
                com += inertia_trans.translation.vector * link.inertial.mass.clone();
                total_mass += link.inertial.mass.clone();
            }
        }
    });
    com / total_mass
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::link::*;
    use crate::node::*;
    use na::*;
    #[cfg(target_arch = "wasm32")]
    use wasm_bindgen_test::wasm_bindgen_test as test;

    #[test]
    fn test_update_center_of_mass() {
        let j0 = NodeBuilder::new()
            .translation(Translation3::new(0.0, 1.0, 0.0))
            .into_node();
        let j1 = NodeBuilder::new()
            .translation(Translation3::new(0.0, 0.0, 1.0))
            .joint_type(JointType::Rotational {
                axis: Vector3::y_axis(),
            })
            .into_node();
        j0.set_link(Some(
            LinkBuilder::new()
                .name("l0")
                .inertial(Inertial::new(Isometry3::identity(), 1.0, Matrix3::zeros()))
                .finalize(),
        ));
        let mut i1 = Inertial::new(Isometry3::identity(), 4.0, Matrix3::zeros());
        i1.set_origin(Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0),
            UnitQuaternion::identity(),
        ));
        j1.set_link(Some(LinkBuilder::new().name("l1").inertial(i1).finalize()));
        j1.set_parent(&j0);
        let tree = Chain::from_root(j0);
        let com1 = center_of_mass(&tree);
        assert!((com1.x - 0.0f64).abs() < f64::EPSILON);
        assert!((com1.y - 1.0f64).abs() < f64::EPSILON);
        assert!((com1.z - 1.6f64).abs() < f64::EPSILON);
        j1.set_joint_position(0.5).unwrap();
        let com2 = center_of_mass(&tree);
        assert!((com2.x - 0.383540).abs() < 0.0001);
        assert!((com2.y - 1.0f64).abs() < f64::EPSILON);
        assert!((com2.z - 1.502066).abs() < 0.0001);
    }
}
