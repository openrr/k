/*
  Copyright 2020 Takashi Ogura

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
use nalgebra::RealField;

/// Information for copying joint state of other joint
///
/// For example, `Mimic` is used to calculate the position of the gripper(R) from
/// gripper(L). In that case, the code like below will be used.
///
/// ```
/// let mimic_for_gripper_r = k::joint::Mimic::new(-1.0, 0.0);
/// ```
///
/// output position (mimic_position() is calculated by `joint positions = joint[name] * multiplier + origin`
///
#[derive(Debug, Clone)]
pub struct Mimic<T: RealField> {
    pub multiplier: T,
    pub origin: T,
}

impl<T> Mimic<T>
where
    T: RealField,
{
    /// Create new instance of Mimic
    ///
    /// # Examples
    ///
    /// ```
    /// let m = k::joint::Mimic::<f64>::new(1.0, 0.5);
    /// ```
    pub fn new(multiplier: T, origin: T) -> Self {
        Mimic { multiplier, origin }
    }
    /// Calculate the mimic joint position
    ///
    /// # Examples
    ///
    /// ```
    /// let m = k::joint::Mimic::<f64>::new(1.0, 0.5);
    /// assert_eq!(m.mimic_position(0.2), 0.7); // 0.2 * 1.0 + 0.5
    /// ```
    ///
    /// ```
    /// let m = k::joint::Mimic::<f64>::new(-2.0, -0.4);
    /// assert_eq!(m.mimic_position(0.2), -0.8); // 0.2 * -2.0 - 0.4
    /// ```
    pub fn mimic_position(&self, from_position: T) -> T {
        from_position * self.multiplier + self.origin
    }
}
