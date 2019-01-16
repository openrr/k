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
//! `link` can be used to show the shape of the robot, or collision checking libraries.
//!
//! `link` module is optional for `k`.
//!
use na::{Isometry3, Matrix3, Real, Vector3};

#[derive(Debug, Clone)]
pub enum Geometry<T: Real> {
    Box { depth: T, width: T, height: T },
    Cylinder { radius: T, length: T },
    Sphere { radius: T },
    Mesh { filename: String, scale: Vector3<T> },
}

#[derive(Debug, Default, Clone)]
pub struct Color<T: Real> {
    pub r: T,
    pub g: T,
    pub b: T,
    pub a: T,
}

#[derive(Debug, Clone)]
pub struct Texture {
    pub filename: String,
}

impl Default for Texture {
    fn default() -> Texture {
        Texture {
            filename: "".to_string(),
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Material<T: Real> {
    pub name: String,
    pub color: Color<T>,
    pub texture: Texture,
}

#[derive(Debug, Clone)]
pub struct Inertial<T: Real> {
    origin: Isometry3<T>,
    pub mass: T,
    pub inertia: Matrix3<T>,
    world_transform_cache: Option<Isometry3<T>>,
}

// TODO
impl<T> Inertial<T>
where
    T: Real,
{
    pub fn from_mass(mass: T) -> Self {
        Self {
            origin: Isometry3::identity(),
            mass,
            inertia: Matrix3::identity(),
            world_transform_cache: None,
        }
    }
    pub fn new(origin: Isometry3<T>, mass: T, inertia: Matrix3<T>) -> Self {
        Self {
            origin,
            mass,
            inertia,
            world_transform_cache: None,
        }
    }
    pub fn set_origin(&mut self, origin: Isometry3<T>) {
        self.origin = origin;
        self.world_transform_cache = None;
    }
    pub fn origin(&self) -> &Isometry3<T> {
        &self.origin
    }
    pub fn clear_world_transform(&mut self) {
        self.world_transform_cache = None
    }
    pub fn set_world_transform(&mut self, trans: Isometry3<T>) {
        self.world_transform_cache = Some(trans);
    }
    pub fn world_transform(&self) -> &Option<Isometry3<T>> {
        &self.world_transform_cache
    }
}

#[derive(Debug, Clone)]
pub struct Visual<T: Real> {
    pub name: String,
    origin: Isometry3<T>,
    pub geometry: Geometry<T>,
    pub material: Material<T>,
    world_transform_cache: Option<Isometry3<T>>,
}

impl<T> Visual<T>
where
    T: Real,
{
    pub fn new(
        name: String,
        origin: Isometry3<T>,
        geometry: Geometry<T>,
        material: Material<T>,
    ) -> Self {
        Self {
            name,
            origin,
            geometry,
            material,
            world_transform_cache: None,
        }
    }
    pub fn set_origin(&mut self, origin: Isometry3<T>) {
        self.origin = origin;
        self.world_transform_cache = None;
    }
    pub fn origin(&self) -> &Isometry3<T> {
        &self.origin
    }
    pub fn clear_world_transform(&mut self) {
        self.world_transform_cache = None
    }
    pub fn set_world_transform(&mut self, trans: Isometry3<T>) {
        self.world_transform_cache = Some(trans);
    }
    pub fn world_transform(&self) -> &Option<Isometry3<T>> {
        &self.world_transform_cache
    }
}

#[derive(Debug, Clone)]
pub struct Collision<T: Real> {
    pub name: String,
    origin: Isometry3<T>,
    pub geometry: Geometry<T>,
    world_transform_cache: Option<Isometry3<T>>,
}

impl<T> Collision<T>
where
    T: Real,
{
    pub fn new(name: String, origin: Isometry3<T>, geometry: Geometry<T>) -> Self {
        Self {
            name,
            origin,
            geometry,
            world_transform_cache: None,
        }
    }
    pub fn set_origin(&mut self, origin: Isometry3<T>) {
        self.origin = origin;
        self.world_transform_cache = None;
    }
    pub fn origin(&self) -> &Isometry3<T> {
        &self.origin
    }
    pub fn clear_world_transform(&mut self) {
        self.world_transform_cache = None
    }
    pub fn set_world_transform(&mut self, trans: Isometry3<T>) {
        self.world_transform_cache = Some(trans);
    }
    pub fn world_transform(&self) -> &Option<Isometry3<T>> {
        &self.world_transform_cache
    }
}

#[derive(Debug, Clone)]
pub struct Link<T: Real> {
    pub name: String,
    pub inertial: Inertial<T>,
    pub visuals: Vec<Visual<T>>,
    pub collisions: Vec<Collision<T>>,
}

impl<T> Default for Link<T>
where
    T: Real,
{
    fn default() -> Self {
        Self {
            name: "".to_owned(),
            inertial: Inertial::new(Isometry3::identity(), T::zero(), Matrix3::identity()),
            visuals: Vec::new(),
            collisions: Vec::new(),
        }
    }
}

pub struct LinkBuilder<T>
where
    T: Real,
{
    name: String,
    inertial: Inertial<T>,
    visuals: Vec<Visual<T>>,
    collisions: Vec<Collision<T>>,
}

impl<T> LinkBuilder<T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            name: "".to_owned(),
            inertial: Inertial::new(Isometry3::identity(), T::zero(), Matrix3::identity()),
            visuals: Vec::new(),
            collisions: Vec::new(),
        }
    }
    pub fn name(mut self, name: &str) -> Self {
        self.name = name.to_owned();
        self
    }
    pub fn inertial(mut self, inertial: Inertial<T>) -> Self {
        self.inertial = inertial;
        self
    }
    pub fn add_visual(mut self, visual: Visual<T>) -> Self {
        self.visuals.push(visual);
        self
    }
    pub fn add_collision(mut self, collision: Collision<T>) -> Self {
        self.collisions.push(collision);
        self
    }
    pub fn finalize(self) -> Link<T> {
        Link {
            name: self.name,
            inertial: self.inertial,
            visuals: self.visuals,
            collisions: self.collisions,
        }
    }
}

impl<T> Default for LinkBuilder<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}
