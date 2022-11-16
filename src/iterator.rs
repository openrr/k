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
//! Iterators to iterate descendants and ancestors
use super::node::*;
use nalgebra::RealField;
use simba::scalar::SubsetOf;
use std::collections::VecDeque;

#[derive(Debug)]
/// Iterator for parents
pub struct Ancestors<T>
where
    T: RealField,
{
    parent: Option<Node<T>>,
}

impl<T> Ancestors<T>
where
    T: RealField,
{
    pub fn new(parent: Option<Node<T>>) -> Self {
        Self { parent }
    }
}

impl<T> Iterator for Ancestors<T>
where
    T: RealField + SubsetOf<f64>,
{
    type Item = Node<T>;
    #[allow(clippy::question_mark)]
    fn next(&mut self) -> Option<Node<T>> {
        if self.parent.is_none() {
            return None;
        }
        let next = self.parent.clone().unwrap();
        self.parent = next.parent();
        Some(next)
    }
}

#[derive(Debug)]
/// Iterator for children
pub struct Descendants<T>
where
    T: RealField,
{
    stack: VecDeque<Node<T>>,
}

impl<T> Descendants<T>
where
    T: RealField,
{
    pub fn new(stack: Vec<Node<T>>) -> Self {
        Self {
            stack: stack.into(),
        }
    }
}

impl<T> Iterator for Descendants<T>
where
    T: RealField + SubsetOf<f64>,
{
    type Item = Node<T>;

    fn next(&mut self) -> Option<Self::Item> {
        let node = match self.stack.pop_front() {
            Some(node) => node,
            None => {
                return None;
            }
        };
        self.stack.extend(node.children().clone());
        Some(node)
    }
}
