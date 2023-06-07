use std::{sync::Arc, hash::Hash};

use ddo::Dominance;

use super::model::AlpState;

pub struct AlpKey(Arc<AlpState>);
impl Hash for AlpKey {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.0.rem.hash(state);
        self.0.info.iter().for_each(|i| i.prev_class.hash(state));
    }
}

impl PartialEq for AlpKey {
    fn eq(&self, other: &Self) -> bool {
        if self.0.rem != other.0.rem {
            return false;
        }
        self.0.info.iter()
            .zip(other.0.info.iter())
            .all(|(i1, i2)| i1.prev_class == i2.prev_class)
    }
}

impl Eq for AlpKey {}

pub struct AlpDominance;
impl Dominance for AlpDominance {
    type State = AlpState;
    type Key = AlpKey;

    fn get_key(&self, state: Arc<Self::State>) -> Option<Self::Key> {
        Some(AlpKey(state))
    }

    fn nb_dimensions(&self, state: &Self::State) -> usize {
        state.info.len()
    }

    fn get_coordinate(&self, state: &Self::State, i: usize) -> isize {
        - state.info[i].prev_time
    }

    fn use_value(&self) -> bool { true }
}