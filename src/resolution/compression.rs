use std::collections::HashMap;

use clustering::{kmeans, Elem};
use ddo::{Compression, Problem, Decision, Dominance};

use crate::instance::AlpInstance;

use super::model::{Alp, AlpState, RunwayState, AlpDecision};

struct Item<'a> {
    id: usize,
    pb: &'a Alp,
}

impl<'a> Elem for Item<'a> {
    fn dimensions(&self) -> usize {
        self.pb.instance.nb_classes
    }

    fn at(&self, i: usize) -> f64 {
        self.pb.instance.separation[self.id][i] as f64
    }
}

pub struct AlpCompression<'a> {
    pub problem: &'a Alp,
    pub meta_problem: Alp,
    pub class_membership: Vec<usize>,
    pub decision_membership: HashMap<isize, isize>,
}

impl<'a> AlpCompression<'a> {
    pub fn new(problem: &'a Alp, n_meta_classes: usize) -> Self {
        let mut elems = vec![];
        for i in 0..problem.instance.nb_classes {
            elems.push(Item {
                id: i,
                pb: problem,
            });
        }
        let clustering = kmeans(n_meta_classes, Some(0), &elems, 1000);

        let classes = Self::compute_meta_classes(problem, &clustering.membership);
        let separation = Self::compute_meta_separation(problem, &clustering.membership, n_meta_classes);

        let meta_instance = AlpInstance {
            nb_classes: n_meta_classes,
            nb_aircrafts: problem.instance.nb_aircrafts,
            nb_runways: problem.instance.nb_runways,
            classes,
            target: problem.instance.target.clone(),
            latest: problem.instance.latest.clone(),
            separation,
        };
        let meta_problem = Alp::new(meta_instance);

        let mut decision_membership = HashMap::new();
        for (i, j) in clustering.membership.iter().copied().enumerate() {
            for r in 0..problem.instance.nb_runways {
                decision_membership.insert(
                    problem.to_decision(&AlpDecision { class: i, runway: r }),
                    meta_problem.to_decision(&AlpDecision { class: j, runway: r })
                );
            }
        }
        decision_membership.insert(-1, -1);

        AlpCompression {
            problem,
            meta_problem,
            class_membership: clustering.membership,
            decision_membership,
        }
    }

    fn compute_meta_classes(pb: &Alp, membership: &Vec<usize>) -> Vec<usize> {
        pb.instance.classes.iter().map(|c| membership[*c]).collect()
    }

    fn compute_meta_separation(pb: &Alp, membership: &Vec<usize>, n_meta_classes: usize) -> Vec<Vec<isize>> {
        let mut meta_separation = vec![vec![isize::MAX; n_meta_classes]; n_meta_classes];

        for (i, a) in membership.iter().copied().enumerate() {
            for (j, b) in membership.iter().copied().enumerate() {
                meta_separation[a][b] = meta_separation[a][b].min(pb.instance.separation[i][j]);
            }
        }

        meta_separation
    }
}

impl<'a> Compression for AlpCompression<'a> {
    type State = AlpState;

    fn get_compressed_problem(&self) -> &dyn Problem<State = Self::State> {
        &self.meta_problem
    }

    fn compress(&self, state: &AlpState) -> AlpState {
        let mut rem = vec![0; self.meta_problem.instance.nb_classes];
        for (class, r) in state.rem.iter().copied().enumerate() {
            rem[self.class_membership[class]] += r;
        }

        let mut info = vec![];
        for rs in state.info.iter() {
            if rs.prev_class == -1 {
                info.push(RunwayState { prev_time: rs.prev_time, prev_class: -1 });
            } else {
                info.push(RunwayState { prev_time: rs.prev_time, prev_class: self.class_membership[rs.prev_class as usize] as isize });
            }
        }

        AlpState { rem, info }
    }

    fn decompress(&self, solution: &Vec<Decision>) -> Vec<Decision> {
        solution.clone()
    }
}

#[derive(PartialEq, Eq, Hash)]
pub struct AlpKey {
    /// The number of remaining aircrafts to schedule for each class
    pub rem: Vec<usize>,
    /// The aircraft class scheduled the latest
    pub prev_class: Vec<isize>,
}

#[derive(PartialEq, Eq, PartialOrd, Ord)]
pub struct AlpValue {
    /// The sum of all prev times (negated)
    pub tot: isize,
    /// The time of the latest aircraft scheduled (negated)
    pub prev_times: Vec<isize>,
}

pub struct AlpDominance;
impl Dominance for AlpDominance {
    type State = AlpState;
    type Key = AlpKey;
    type Value = AlpValue;

    fn get_key(&self, state: &Self::State) -> Self::Key {
        AlpKey {
            rem: state.rem.clone(),
            prev_class: state.info.iter().map(|i| i.prev_class).collect(),
        }
    }

    fn get_value(&self, state: &Self::State) -> Self::Value {
        let mut tot = 0;
        let mut prev_times = vec![];

        for i in state.info.iter() {
            tot -= i.prev_time;
            prev_times.push(-i.prev_time);
        }

        AlpValue {
            tot,
            prev_times,
        }
    }

    fn is_dominated_by(&self, a: &Self::Value, b: &Self::Value) -> bool {
        if -a.tot < -b.tot {
            return false;
        }

        for i in 0..a.prev_times.len() {
            if -a.prev_times[i] < -b.prev_times[i] {
                return false;
            }
        }

        true
    }
}