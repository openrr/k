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
use std::rc::{Rc, Weak};
use std::slice::Iter;
use std::cell::{Ref, RefCell, RefMut};


pub type RcNode<T> = Rc<RefCell<NodeImpl<T>>>;
type WeakNode<T> = Weak<RefCell<NodeImpl<T>>>;

#[derive(Debug, Clone)]
/// Generic Node for tree struct
pub struct NodeImpl<T> {
    pub parent: Option<WeakNode<T>>,
    pub children: Vec<Node<T>>,
    pub data: T,
}

#[derive(Debug)]
pub struct Node<T>(Rc<RefCell<NodeImpl<T>>>);

impl<T> Node<T> {
    pub fn new(obj: T) -> Self {
        Node::<T>(Rc::new(RefCell::new(NodeImpl {
            parent: None,
            children: Vec::new(),
            data: obj,
        })))
    }
    pub fn borrow<'a>(&'a self) -> Ref<'a, NodeImpl<T>> {
        self.0.borrow()
    }
    pub fn borrow_mut<'a>(&'a self) -> RefMut<'a, NodeImpl<T>> {
        self.0.borrow_mut()
    }

    /// iter from the end to root, it contains nodes[id] itsself
    pub fn iter_ancestors(&self) -> Ancestors<T> {
        Ancestors { parent: Some(self.clone()) }
    }
    pub fn iter_descendants(&self) -> Descendants<T> {
        Descendants { stack: vec![self.clone()] }
    }

    /// Set parent and child relations at same time
    pub fn set_parent(&self, parent: &Node<T>) {
        self.borrow_mut().parent = Some(Rc::downgrade(&parent.0));
        parent.borrow_mut().children.push(self.clone());
    }
}

impl<T> ::std::clone::Clone for Node<T> {
    fn clone(&self) -> Self {
        Node::<T>(self.0.clone())
    }
}

pub struct NodeIter<'a, T: 'a> {
    pub iter: Iter<'a, Node<T>>,
}

impl<'a, T: 'a> Iterator for NodeIter<'a, T> {
    type Item = Ref<'a, T>;

    fn next(&mut self) -> Option<Ref<'a, T>> {
        self.iter.next().map(|rc| {
            Ref::map(rc.borrow(), |node| &node.data)
        })
    }
}

pub struct NodeIterMut<'a, T: 'a> {
    pub iter: Iter<'a, Node<T>>,
}

impl<'a, T: 'a> Iterator for NodeIterMut<'a, T> {
    type Item = RefMut<'a, T>;

    fn next(&mut self) -> Option<RefMut<'a, T>> {
        self.iter.next().map(|rc| {
            RefMut::map(rc.borrow_mut(), |node| &mut node.data)
        })
    }
}

pub struct Ancestors<T> {
    parent: Option<Node<T>>,
}

impl<T> Iterator for Ancestors<T> {
    type Item = Node<T>;

    fn next(&mut self) -> Option<Node<T>> {
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

pub struct Descendants<T> {
    stack: Vec<Node<T>>,
}

impl<T> Iterator for Descendants<T> {
    type Item = Node<T>;

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
    let n0 = Node::new(0);
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
