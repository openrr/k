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

/// min/max range to check the joint position
#[derive(Copy, Debug, Clone)]
pub struct Range<T: RealField> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
where
    T: RealField,
{
    /// Create new Range instance
    ///
    /// In case `min` is greater than `max`, this function panics.
    ///
    /// # Examples
    ///
    /// ```
    /// let range = k::joint::Range::new(-1.0, 1.0);
    /// // let range = k::joint::Range::new(1.0, -1.0);  // panic
    /// ```
    pub fn new(min: T, max: T) -> Self {
        assert!(min <= max, "min must be less than or equal to max");
        Range { min, max }
    }
    /// Check if the value is in the range
    ///
    /// `true` means it is OK.
    /// If the val is the same as the limit value (`min` or `max`), it returns true (valid).
    ///
    /// # Examples
    ///
    /// ```
    /// let range = k::joint::Range::new(-1.0, 1.0);
    /// assert!(range.is_valid(0.0));
    /// assert!(range.is_valid(1.0));
    /// assert!(!range.is_valid(1.5));
    /// ```
    pub fn is_valid(&self, val: T) -> bool {
        val <= self.max && val >= self.min
    }
}

impl<T> From<::std::ops::RangeInclusive<T>> for Range<T>
where
    T: RealField,
{
    /// # Examples
    ///
    /// ```
    /// let range : k::joint::Range<f64> = (-1.0..=1.0).into();
    /// assert!(range.is_valid(0.0));
    /// assert!(range.is_valid(1.0));
    /// assert!(!range.is_valid(1.5));
    /// ```
    fn from(range: ::std::ops::RangeInclusive<T>) -> Self {
        let (min, max) = range.into_inner();
        Range::new(min, max)
    }
}
