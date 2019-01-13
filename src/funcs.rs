use chain::*;
use joint::*;
use na::{DMatrix, Real, Vector3};

/// Calculate Jacobian of the serial chain (manipulator).
pub fn jacobian<T>(arm: &SerialChain<T>) -> DMatrix<T>
where
    T: Real,
{
    let dof = arm.dof();
    let t_n = arm.end_transform();
    arm.update_transforms();
    let p_n = t_n.translation;
    let jacobi_vec = arm
        .iter_joints()
        .map(|joint| {
            let t_i = joint.world_transform().unwrap();
            match joint.joint_type {
                JointType::Linear { axis } => {
                    let p_i = t_i.rotation * axis;
                    [p_i[0], p_i[1], p_i[2], na::zero(), na::zero(), na::zero()]
                }
                JointType::Rotational { axis } => {
                    let p_i = t_i.translation;
                    let a_i = t_i.rotation * axis;
                    let dp_i = a_i.cross(&(p_n.vector - p_i.vector));
                    [dp_i[0], dp_i[1], dp_i[2], a_i[0], a_i[1], a_i[2]]
                }
                JointType::Fixed => panic!("impossible, bug of jacobian"),
            }
        })
        .collect::<Vec<_>>();
    // Pi: a_i x (p_n - Pi)
    // wi: a_i
    DMatrix::from_fn(6, dof, |r, c| jacobi_vec[c][r])
}

/// Calculate the center of mass of the chain
///
/// ```
/// use k::*;
/// use k::link::*;
///
/// let j0 = JointBuilder::new()
///     .translation(Translation3::new(0.0, 1.0, 0.0))
///     .into_node();
/// let j1 = JointBuilder::new()
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
    T: Real,
{
    let mut total_mass = T::zero();
    let mut com = Vector3::zeros();

    chain.update_transforms();
    chain.iter().for_each(|node| {
        if let Some(trans) = node.joint().world_transform() {
            if let Some(ref link) = *node.link() {
                let inertia_trans = trans * link.inertial.origin().translation;
                com += inertia_trans.translation.vector * link.inertial.mass;
                total_mass += link.inertial.mass;
            }
        }
    });
    com / total_mass
}

#[test]
fn test_update_center_of_mass() {
    use super::joint::*;
    use super::link::*;
    use super::node::*;
    use na::*;
    let j0 = JointBuilder::new()
        .translation(Translation3::new(0.0, 1.0, 0.0))
        .into_node();
    let j1 = JointBuilder::new()
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
    assert_eq!(com1.x, 0.0);
    assert_eq!(com1.y, 1.0);
    assert_eq!(com1.z, 1.6);
    j1.set_joint_position(0.5).unwrap();
    let com2 = center_of_mass(&tree);
    assert!((com2.x - 0.383540).abs() < 0.0001);
    assert_eq!(com2.y, 1.0);
    assert!((com2.z - 1.502066).abs() < 0.0001);
}
