use std::collections::HashMap;

use clustering::{kmeans, Elem};
use ddo::{Compression, Problem, Decision};

use crate::instance::AlpInstance;

use super::model::{Alp, AlpState, RunwayState};

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
    pub membership: HashMap<isize, isize>,
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

        let mut membership = HashMap::new();
        for (i, j) in clustering.membership.iter().enumerate() {
            membership.insert(i as isize, *j as isize);
        }
        membership.insert(-1, -1);

        AlpCompression {
            problem,
            meta_problem,
            membership,
        }
    }

    fn compute_meta_classes(pb: &Alp, membership: &Vec<usize>) -> Vec<usize> {
        pb.instance.classes.iter().map(|c| membership[*c]).collect()
    }

    fn compute_meta_separation(pb: &Alp, membership: &Vec<usize>, n_meta_classes: usize) -> Vec<Vec<isize>> {
        let mut meta_separation = vec![vec![isize::MAX; n_meta_classes]; n_meta_classes];

        for (i, a) in membership.iter().copied().enumerate() {
            for (j, b) in membership.iter().copied().enumerate() {
                if a != b {
                    meta_separation[a][b] = meta_separation[a][b].min(pb.instance.separation[i][j]);
                }
            }
        }

        for i in 0..n_meta_classes {
            meta_separation[i][i] = 0;
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
        for (c, r) in state.rem.iter().copied().enumerate() {
            rem[self.membership[&(c as isize)] as usize] += r;
        }

        let info = state.info.iter().map(|s| RunwayState { prev_time: s.prev_time, prev_class: self.membership[&s.prev_class] }).collect::<Vec<RunwayState>>();
        let mut info_sorted = info.clone();
        info_sorted.sort_unstable();
        
        AlpState {
            rem,
            info,
            info_sorted,
        }
    }

    fn decompress(&self, solution: &Vec<Decision>) -> Vec<Decision> {
        solution.clone()
    }
}