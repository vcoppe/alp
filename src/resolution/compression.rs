use std::{collections::{HashMap, BTreeSet, hash_map::Entry, HashSet}, mem::swap, ops::Bound::*, vec};

use clustering::{kmeans, Elem};
use ddo::{Compression, Problem, Decision};

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
    pub states: HashMap<(Vec<usize>,Vec<isize>), BTreeSet<(isize, Vec<isize>)>>,
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

        let states = Self::compute_meta_states(&meta_problem);

        AlpCompression {
            problem,
            meta_problem,
            class_membership: clustering.membership,
            decision_membership,
            states,
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

    fn compute_meta_states(meta_pb: &Alp) -> HashMap<(Vec<usize>,Vec<isize>), BTreeSet<(isize, Vec<isize>)>> {
        let mut map: HashMap<(Vec<usize>,Vec<isize>), BTreeSet<(isize, Vec<isize>)>> = HashMap::new();

        let mut depth = 0;
        let mut current = HashSet::new();
        let mut next = HashSet::new();

        current.insert(meta_pb.initial_state());

        while let Some(var) = meta_pb.next_variable(depth, &mut current.iter()) {
            for state in current.drain() {
                
                let mut prev_times = vec![];
                let mut prev_classes = vec![];
                let mut sum = 0;

                for rs in state.info.iter() {
                    prev_times.push(rs.prev_time);
                    prev_classes.push(rs.prev_class);
                    sum += rs.prev_time;
                }

                match map.entry((state.rem.clone(), prev_classes)) {
                    Entry::Occupied(mut e) => {
                        e.get_mut().insert((sum, prev_times));
                    },
                    Entry::Vacant(e) => {
                        let mut set = BTreeSet::new();
                        set.insert((sum, prev_times));
                        e.insert(set);
                    },
                }

                meta_pb.for_each_in_domain(var, &state, &mut |d| { next.insert(meta_pb.transition(&state, d)); });
            }

            swap(&mut current, &mut next);
            depth += 1;
        }

        map
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

        let mut prev_times = vec![];
        let mut prev_classes = vec![];
        let mut sum = 0;

        for rs in state.info.iter() {
            prev_times.push(rs.prev_time);
            if rs.prev_class == -1 {
                prev_classes.push(-1);

            } else {
                prev_classes.push(self.class_membership[rs.prev_class as usize] as isize);
            }
            sum += rs.prev_time;
        }

        let key = (rem, prev_classes);

        match self.states.get(&key) {
            Some(infos) => {
                let range = infos.range((Unbounded, Included((sum, prev_times))));
                for (_, precomputed_times) in range.rev() {
                    let mut dominated = true;
                    for r in 0..self.problem.instance.nb_runways {
                        if state.info[r].prev_time < precomputed_times[r] {
                            dominated = false;
                            break;
                        }
                    }

                    if dominated {
                        return AlpState {
                            rem: key.0,
                            info: (0..self.problem.instance.nb_runways).map(|r| RunwayState { prev_time: precomputed_times[r], prev_class: key.1[r] }).collect(),
                        };
                    }
                }
                AlpState { rem: key.0, info: vec![] }
            },
            None => AlpState { rem: key.0, info: vec![] },
        }
    }

    fn decompress(&self, solution: &Vec<Decision>) -> Vec<Decision> {
        solution.clone()
    }
}