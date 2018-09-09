/*
   Copyright 2017 Takashi Ogura

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */
//! Abstruct tree structure using `Rc<RefCell<T>>`
use std::cell::{Ref, RefCell, RefMut};
use std::rc::{Rc, Weak};

type WeakNode<T, K> = Weak<RefCell<NodeImpl<T, K>>>;

#[derive(Debug, Clone)]
/// Generic Node for tree struct
pub struct NodeImpl<T, K> {
    pub parent: Option<WeakNode<T, K>>,
    pub children: Vec<Node<T, K>>,
    pub data: T,
    pub sub_parent: Option<WeakNode<T, K>>,
    pub sub_children: Vec<Node<T, K>>,
    pub sub_data: Option<K>,
}

#[derive(Debug)]
/// Graph Node, which has a parent and children
pub struct Node<T, K>(Rc<RefCell<NodeImpl<T, K>>>);

impl<T, K> Node<T, K> {
    pub fn new(obj: T) -> Self {
        Node::<T, K>(Rc::new(RefCell::new(NodeImpl {
            parent: None,
            children: Vec::new(),
            data: obj,
            sub_parent: None,
            sub_children: Vec::new(),
            sub_data: None,
        })))
    }
    pub fn borrow<'a>(&'a self) -> Ref<'a, NodeImpl<T, K>> {
        self.0.borrow()
    }
    pub fn borrow_mut<'a>(&'a self) -> RefMut<'a, NodeImpl<T, K>> {
        self.0.borrow_mut()
    }

    /// iter from the end to root, it contains nodes[id] itsself
    pub fn iter_ancestors(&self) -> Ancestors<T, K> {
        Ancestors {
            parent: Some(self.clone()),
        }
    }
    /// iter to the end, it contains nodes[id] itsself
    pub fn iter_descendants(&self) -> Descendants<T, K> {
        Descendants {
            stack: vec![self.clone()],
        }
    }

    /// Set parent and child relations at same time
    pub fn set_parent(&self, parent: &Node<T, K>) {
        self.borrow_mut().parent = Some(Rc::downgrade(&parent.0));
        parent.borrow_mut().children.push(self.clone());
    }

    /// Set mimic parent
    pub fn set_sub_parent(&self, parent: &Node<T, K>) {
        self.borrow_mut().sub_parent = Some(Rc::downgrade(&parent.0));
        parent.borrow_mut().sub_children.push(self.clone());
    }
}

impl<T, K> ::std::clone::Clone for Node<T, K> {
    fn clone(&self) -> Self {
        Node::<T, K>(self.0.clone())
    }
}

impl<T, K> PartialEq for Node<T, K> {
    fn eq(&self, other: &Node<T, K>) -> bool {
        &*self.0 as *const RefCell<NodeImpl<T, K>> == &*other.0 as *const RefCell<NodeImpl<T, K>>
    }
}

#[derive(Debug)]
/// Iterator for parents
pub struct Ancestors<T, K> {
    parent: Option<Node<T, K>>,
}

impl<T, K> Iterator for Ancestors<T, K> {
    type Item = Node<T, K>;

    fn next(&mut self) -> Option<Node<T, K>> {
        if self.parent.is_none() {
            return None;
        }
        let next = self.parent.clone().unwrap();
        match next.borrow().parent {
            None => self.parent = None,
            Some(ref parent) => {
                self.parent = Some(Node(parent.upgrade().expect("failed to get parent")))
            }
        }
        Some(next)
    }
}

#[derive(Debug)]
/// Iterator for children
pub struct Descendants<T, K> {
    stack: Vec<Node<T, K>>,
}

impl<T, K> Iterator for Descendants<T, K> {
    type Item = Node<T, K>;

    fn next(&mut self) -> Option<Self::Item> {
        let node = match self.stack.pop() {
            Some(node) => node,
            None => {
                return None;
            }
        };
        self.stack.extend(node.borrow().children.clone());
        Some(node)
    }
}

#[test]
fn test() {
    let n0 = Node::<i32, i32>::new(0);
    let n1 = Node::new(1);
    let n2 = Node::new(2);
    let n3 = Node::new(3);
    let n4 = Node::new(4);
    let n5 = Node::new(5);
    let n6 = Node::new(6);
    let n7 = Node::new(7);
    n1.set_parent(&n0);
    n2.set_parent(&n1);
    n3.set_parent(&n2);
    n4.set_parent(&n3);
    n5.set_parent(&n4);
    n6.set_parent(&n2);
    n7.set_parent(&n6);
    // 0 1 2 3 4 5
    //       6 7
    assert_eq!(
        n0.iter_descendants()
            .map(|ref_node| ref_node.borrow().data)
            .collect::<Vec<_>>(),
        vec![0, 1, 2, 6, 7, 3, 4, 5]
    );
    assert_eq!(
        n6.iter_descendants()
            .map(|ref_node| ref_node.borrow().data)
            .collect::<Vec<_>>(),
        vec![6, 7]
    );
    assert_eq!(
        n7.iter_ancestors()
            .map(|ref_node| ref_node.borrow().data)
            .collect::<Vec<_>>(),
        vec![7, 6, 2, 1, 0]
    );
}
