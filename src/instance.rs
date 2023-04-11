//! This module defines an abstract representation of a ALP instance.

use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlpInstance {
    pub nb_classes: usize,
    pub nb_aircrafts: usize,
    pub nb_runways: usize,
    pub classes: Vec<usize>,
    pub target: Vec<isize>,
    pub latest: Vec<isize>,
    pub separation: Vec<Vec<isize>>,
}
