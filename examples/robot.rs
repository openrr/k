extern crate kiss3d;
extern crate k;
extern crate alga;
use k::*;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use alga::general::Real;

extern crate nalgebra as na;
use na::{Isometry3, Vector3, UnitQuaternion, Translation3};

fn create_joint_with_link_array(name: &str) -> VecKinematicChain<f32> {
    let j1 = Joint::new("lj1", JointType::Rotational { axis: Vector3::x_axis() });
    let j2 = Joint::new("lj2", JointType::Linear { axis: Vector3::y_axis() });
    let j3 = Joint::new("lj3", JointType::Rotational { axis: Vector3::x_axis() });
    let mut joint_with_link1 = Link::new("llink1", j1);
    joint_with_link1.transform = Isometry3::from_parts(Translation3::new(0.0, 0.2, 0.0),
                                                       UnitQuaternion::identity());
    let mut joint_with_link2 = Link::new("link2", j2);
    joint_with_link2.transform = Isometry3::from_parts(Translation3::new(0.0, 0.2, 0.0),
                                                       UnitQuaternion::identity());
    let mut joint_with_link3 = Link::new("link2", j3);
    joint_with_link3.transform = Isometry3::from_parts(Translation3::new(0.0, 0.2, 0.0),
                                                       UnitQuaternion::identity());
    VecKinematicChain::new(name,
                           vec![joint_with_link1, joint_with_link2, joint_with_link3])
}

fn create_cubes(window: &mut Window) -> Vec<SceneNode> {
    let mut c1 = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);
    let mut c2 = window.add_cube(0.1, 0.1, 0.1);
    c2.set_color(0.0, 1.0, 0.0);
    let mut c3 = window.add_cube(0.1, 0.1, 0.1);
    c3.set_color(0.0, 0.0, 1.0);
    let mut c4 = window.add_cube(0.1, 0.1, 0.1);
    c4.set_color(1.0, 0.0, 1.0);
    vec![c1, c2, c3, c4]
}

fn main() {
    let mut lleg = create_joint_with_link_array("left_leg");
    lleg.transform = Isometry3::from_parts(Translation3::new(0.2, 0.2, 0.0),
                                           UnitQuaternion::identity());

    let mut rleg = create_joint_with_link_array("right_leg");
    rleg.transform = Isometry3::from_parts(Translation3::new(-0.2, 0.2, 0.0),
                                           UnitQuaternion::identity());
    let mut larm = create_joint_with_link_array("left_arm");
    larm.transform = Isometry3::from_parts(Translation3::new(0.2, -0.2, 0.0),
                                           UnitQuaternion::identity());
    let mut rarm = create_joint_with_link_array("right_arm");
    rarm.transform = Isometry3::from_parts(Translation3::new(-0.2, -0.2, 0.0),
                                           UnitQuaternion::identity());

    let mut rf = LinkStar::new("robo", vec![lleg, rleg, larm, rarm]);

    let mut window = Window::new("k ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = vec![create_cubes(&mut window),
                         create_cubes(&mut window),
                         create_cubes(&mut window),
                         create_cubes(&mut window)];
    let mut root_cube = window.add_cube(0.2, 0.2, 0.2);

    let mut angles = vec![0.0, 0.0, 0.0];
    let mut t = 0.0;
    while window.render() {
        t += 0.1;
        angles[0] = t.sin();
        angles[1] = t.cos() * 0.05;
        angles[2] = t.sin();
        for fr in &mut rf.frames {
            fr.set_joint_angles(&angles).unwrap();
        }
        rf.set_transform(Isometry3::from_parts(Translation3::new(0.0, 0.1 * t.sin(), 0.0),
                                               UnitQuaternion::from_euler_angles(3.14, 0.0, 0.0)));

        for (i, trans_vec) in rf.calc_link_transforms().iter().enumerate() {
            cubes[i][0].set_local_transformation(rf.get_transform() * rf.frames[i].transform);
            for (j, trans) in trans_vec.iter().enumerate() {
                cubes[i][j + 1].set_local_transformation(*trans);
            }
        }
        root_cube.set_local_transformation(rf.get_transform());
    }
}
