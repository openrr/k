/// extern crate k;
extern crate k;
extern crate nalgebra as na;

fn main() {
    let l0 = k::LinkNode::new(
        k::LinkBuilder::new()
            .name("link0")
            .translation(na::Translation3::new(0.0, 0.0, 0.1))
            .joint(
                "link_pitch0",
                k::JointType::Rotational {
                    axis: na::Vector3::y_axis(),
                },
                None,
            )
            .finalize(),
    );
    let l1 = k::LinkNode::new(
        k::LinkBuilder::new()
            .name("link1")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint(
                "link_pitch1",
                k::JointType::Rotational {
                    axis: na::Vector3::y_axis(),
                },
                None,
            )
            .finalize(),
    );

    let l2 = k::LinkNode::new(
        k::LinkBuilder::new()
            .name("hand")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint("fixed", k::JointType::Fixed, None)
            .finalize(),
    );

    let l3 = k::LinkNode::new(
        k::LinkBuilder::new()
            .name("link2")
            .translation(na::Translation3::new(0.0, 0.0, 0.5))
            .joint(
                "link_pitch2",
                k::JointType::Rotational {
                    axis: na::Vector3::y_axis(),
                },
                None,
            )
            .finalize(),
    );

    l1.set_parent(&l0);
    l2.set_parent(&l1);
    l3.set_parent(&l0);

    let tree = k::LinkTree::from_root("tree0", l0);
    println!("{}", tree);
}
