use na::{Isometry3, Matrix3, Real, Point3};
//use std::fmt::{self, Display};

//use errors::*;

#[derive(Debug, Clone)]
pub enum Geometry<T: Real> {
    Box { depth: T, width: T, height: T },
    Cylinder { radius: T, length: T },
    Sphere { radius: T },
    Mesh { vertices: Vec<Point3<T>>, indices: Vec<usize> },
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
pub enum Element<T: Real> {
    Inertial {
        origin: Isometry3<T>,
        mass: T,
        inertia: Matrix3<T>,
    },
    Visual {
        name: String,
        origin: Isometry3<T>,
        geometry: Geometry<T>,
        material: Material<T>,
    },
    Collision {
        name: String,
        origin: Isometry3<T>,
        geometry: Geometry<T>,
    },
}
