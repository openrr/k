extern crate k;
extern crate nalgebra as na;

fn main() {
    let l0 = k::Node::new(
        k::JointBuilder::new()
            .name("link_pitch0")
            .translation(na::Translation3::new(0.0, 0.0, 0.1))
            .joint_type(k::JointType::Rotational {
                axis: na::Vector3::y_axis(),
            }).finalize(),
    );
    let l1 = k::Node::new(
        k::JointBuilder::new()
            .name("link_pitch1")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint_type(k::JointType::Rotational {
                axis: na::Vector3::y_axis(),
            }).finalize(),
    );

    let l2 = k::Node::new(
        k::JointBuilder::new()
            .name("link_fixed")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint_type(k::JointType::Fixed)
            .finalize(),
    );

    let l3 = k::Node::new(
        k::JointBuilder::new()
            .name("link_pitch2")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint_type(k::JointType::Rotational {
                axis: na::Vector3::y_axis(),
            }).finalize(),
    );

    l1.set_parent(&l0);
    l2.set_parent(&l1);
    l3.set_parent(&l0);

    let tree = k::Chain::from_root(l0);
    println!("{}", tree);
}
