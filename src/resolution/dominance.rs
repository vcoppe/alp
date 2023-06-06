use ddo::Dominance;

use super::model::AlpState;

#[derive(PartialEq, Eq, Hash)]
pub struct AlpKey {
    /// The number of remaining aircrafts to schedule for each class
    pub rem: Vec<usize>,
    /// The aircraft class scheduled the latest for each runway
    pub prev_class: Vec<isize>,
}

pub struct AlpDominance;
impl Dominance for AlpDominance {
    type State = AlpState;
    type Key = AlpKey;

    fn get_key(&self, state: &Self::State) -> Option<Self::Key> {
        Some(AlpKey { rem: state.rem.clone(), prev_class: state.info.iter().map(|i| i.prev_class).collect() })
    }

    fn nb_value_dimensions(&self, state: &Self::State) -> usize {
        state.info.len()
    }

    fn get_value_at(&self, state: &Self::State, i: usize) -> isize {
        - state.info[i].prev_time
    }

    fn use_value(&self) -> bool { true }
}