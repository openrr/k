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
    pub origin: Isometry3<T>,
    pub mass: T,
    pub inertia: Matrix3<T>,
}

// TODO
impl<T> Inertial<T>
where
    T: Real,
{
    pub fn new(mass: T) -> Self {
        Self {
            origin: Isometry3::identity(),
            mass,
            inertia: Matrix3::identity(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Visual<T: Real> {
    pub name: String,
    pub origin: Isometry3<T>,
    pub geometry: Geometry<T>,
    pub material: Material<T>,
}

#[derive(Debug, Clone)]
pub struct Collision<T: Real> {
    pub name: String,
    pub origin: Isometry3<T>,
    pub geometry: Geometry<T>,
}

#[derive(Debug, Clone)]
pub struct Link<T: Real> {
    pub name: String,
    pub inertial: Inertial<T>,
    pub visuals: Vec<Visual<T>>,
    pub collisions: Vec<Collision<T>>,
}

impl<T> Link<T>
where
    T: Real,
{
    pub fn new(name: &str, inertial: Inertial<T>) -> Self {
        Self {
            name: name.to_owned(),
            inertial,
            visuals: Vec::new(),
            collisions: Vec::new(),
        }
    }
}
