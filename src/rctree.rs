use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub type RefNode<T> = Rc<RefCell<Node<T>>>;
type WeakNode<T> = Weak<RefCell<Node<T>>>;

#[derive(Debug, Clone)]
pub struct Node<T> {
    pub parent: Option<WeakNode<T>>,
    pub children: Vec<RefNode<T>>,
    pub data: T,
}

impl<T> Node<T> {
    pub fn new(obj: T) -> Node<T> {
        Node {
            parent: None,
            children: Vec::new(),
            data: obj,
        }
    }
}

/// set parent and child relations at same time
pub fn set_parent_child<T>(parent: &RefNode<T>, child: &RefNode<T>) {
    child.borrow_mut().parent = Some(Rc::downgrade(parent));
    parent.borrow_mut().children.push(child.clone());
}

/// Wrap data: T with RC<RefCell<Node<T>>
pub fn create_ref_node<T>(data: T) -> RefNode<T> {
    Rc::new(RefCell::new(Node::new(data)))
}

// todo: move vec to iter?
pub fn map_descendants<T, F, K>(root: &RefNode<T>, func: &F) -> Vec<K>
    where F: Fn(&RefNode<T>) -> K
{
    let mut ret = Vec::new();
    map_descendants_internal(root, func, &mut ret);
    ret
}

// can be removed..
fn map_descendants_internal<T, F, K>(root: &RefNode<T>, func: &F, ret: &mut Vec<K>)
    where F: Fn(&RefNode<T>) -> K
{
    ret.push(func(root));
    for c in &root.borrow().children {
        map_descendants_internal(c, func, ret);
    }
}

pub fn map_ancestors<T, F, K>(end: &RefNode<T>, func: &F) -> Vec<K>
    where F: Fn(&RefNode<T>) -> K
{
    let mut ret = Vec::new();
    map_ancestors_internal(end, func, &mut ret);
    ret
}

fn map_ancestors_internal<T, F, K>(link: &RefNode<T>, func: &F, ret: &mut Vec<K>)
    where F: Fn(&RefNode<T>) -> K
{
    ret.push(func(link));
    match link.borrow().parent {
        None => {}
        Some(ref parent) => map_ancestors_internal(&Weak::upgrade(parent).unwrap(), func, ret),
    };
}

#[test]
fn test() {
    let n0 = create_ref_node(0);
    let n1 = create_ref_node(1);
    let n2 = create_ref_node(2);
    let n3 = create_ref_node(3);
    let n4 = create_ref_node(4);
    let n5 = create_ref_node(5);
    let n6 = create_ref_node(6);
    let n7 = create_ref_node(7);
    set_parent_child(&n0, &n1);
    set_parent_child(&n1, &n2);
    set_parent_child(&n2, &n3);
    set_parent_child(&n3, &n4);
    set_parent_child(&n4, &n5);
    set_parent_child(&n2, &n6);
    set_parent_child(&n6, &n7);
    assert_eq!(map_descendants(&n0, &|ref_node| ref_node.borrow().data),
               vec![0, 1, 2, 3, 4, 5, 6, 7]);
    assert_eq!(map_descendants(&n6, &|ref_node| ref_node.borrow().data),
               vec![6, 7]);
    assert_eq!(map_ancestors(&n7, &|ref_node| ref_node.borrow().data),
               vec![7, 6, 2, 1, 0]);
}
