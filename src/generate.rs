use std::{time::{SystemTime, UNIX_EPOCH}, fs::File, io::Write};

use clap::Args;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaChaRng;
use rand_distr::{Uniform, Normal, Distribution};

use crate::instance::AlpInstance;

#[derive(Debug, Args)]
pub struct AlpGenerator {
    /// An optional seed to kickstart the instance generation
    #[clap(short='s', long)]
    seed: Option<u128>,
    /// The number of aircrafts
    #[clap(short='n', long, default_value="50")]
    nb_aircrafts: usize,
    /// The number of runways
    #[clap(short='r', long, default_value="5")]
    nb_runways: usize,
    /// The number of aircraft classes
    #[clap(short='k', long, default_value="4")]
    nb_classes: usize,
    /// The number of clusters of similar classes
    #[clap(short='c', long, default_value="2")]
    nb_clusters: usize,
    /// The minimum separation position used to generate the pairwise minimum separation time
    #[clap(long, default_value="100")]
    min_separation_position: isize,
    /// The maximum separation position used to generate the pairwise minimum separation time
    #[clap(long, default_value="200")]
    max_separation_position: isize,
    /// The std deviation of the separation positions among a cluster
    #[clap(long, default_value="10")]
    separation_position_std_dev: isize,
    /// The average time between two aircraft arrivals
    #[clap(long, default_value="50")]
    avg_interarrival_time: isize,
    /// Name of the file where to generate the alp instance
    #[clap(short, long)]
    output: Option<String>,
}

impl AlpGenerator {

    pub fn generate(&mut self) {
        let mut rng = self.rng();

        let mut nb_classes_per_cluster = vec![self.nb_classes / self.nb_clusters; self.nb_clusters];
        for i in 0..(self.nb_classes % self.nb_clusters) {
            nb_classes_per_cluster[i] += 1;
        }
        
        let classes = self.generate_classes(&mut rng);
        let separation = self.generate_separation_costs(&mut rng, &nb_classes_per_cluster);
        let target = self.generate_target(&mut rng);
        let latest = self.generate_latest(&mut rng, &target, &classes);

        let instance = AlpInstance {
            nb_aircrafts: self.nb_aircrafts,
            nb_runways: self.nb_runways,
            nb_classes: self.nb_classes,
            separation,
            classes,
            target,
            latest,
        };

        let instance = serde_json::to_string_pretty(&instance).unwrap();

        if let Some(output) = self.output.as_ref() {
            File::create(output).unwrap().write_all(instance.as_bytes()).unwrap();
        } else {
            println!("{instance}");
        }
    }

    fn generate_separation_costs(&self, rng: &mut impl Rng, nb_classes_per_cluster: &Vec<usize>) -> Vec<Vec<isize>> {
        let mut members = vec![vec![]; self.nb_clusters];
        let mut t = 0_usize;
        for (i, n) in nb_classes_per_cluster.iter().copied().enumerate() {
            for _ in 0..n {
                members[i].push(t);
                t += 1;
            }
        }

        let mut separation_costs = vec![vec![0; self.nb_classes]; self.nb_classes];

        let rand_centroid = Uniform::new_inclusive(self.min_separation_position, self.max_separation_position);
        for a in 0..self.nb_clusters {
            let centroid_a = rand_centroid.sample(rng);

            let rand_position_a = Normal::new(centroid_a as f64, self.separation_position_std_dev as f64).expect("cannot create normal dist");
            let positions_a = (0..nb_classes_per_cluster[a]).map(|_| rand_position_a.sample(rng).round() as isize).collect::<Vec<isize>>();

            for b in 0..self.nb_clusters {
                if a == b {
                    for (i, ti) in members[a].iter().copied().enumerate() {
                        for (j, tj) in members[a].iter().copied().enumerate() {
                            separation_costs[ti][tj] = positions_a[i].abs_diff(positions_a[j]) as isize;
                        }
                    }
                } else {
                    let centroid_b = rand_centroid.sample(rng);
        
                    let rand_position_b = Normal::new(centroid_b as f64, self.separation_position_std_dev as f64).expect("cannot create normal dist");
                    let positions_b = (0..nb_classes_per_cluster[b]).map(|_| rand_position_b.sample(rng).round() as isize).collect::<Vec<isize>>();

                    for (i, ti) in members[a].iter().copied().enumerate() {
                        for (j, tj) in members[b].iter().copied().enumerate() {
                            separation_costs[ti][tj] = positions_a[i].abs_diff(positions_b[j]) as isize;
                        }
                    }
                }
            }
        }
        
        separation_costs
    }

    fn generate_classes(&self, rng: &mut impl Rng) -> Vec<usize> {
        let mut classes = vec![];
        
        let rand_class = Uniform::new(0, self.nb_classes);

        for _ in 0..self.nb_aircrafts {
            classes.push(rand_class.sample(rng));
        }

        classes
    }

    fn generate_target(&self, rng: &mut impl Rng) -> Vec<isize> {
        let mut target = vec![0];

        let rand = Uniform::<f64>::new(0.0, 1.0);

        for i in 1..self.nb_aircrafts {
            target.push(target[i - 1] +  (- rand.sample(rng).ln() * self.avg_interarrival_time as f64).round() as isize);
        }

        target
    }

    fn generate_latest(&self, rng: &mut impl Rng, target: &Vec<isize>, classes: &Vec<usize>) -> Vec<isize> {
        let mut latest = vec![];
        let mut last = vec![0; self.nb_classes];

        let rand = Uniform::new(0, 10 * self.avg_interarrival_time);

        for i in 0..self.nb_aircrafts {
            loop {
                let end = target[i] + rand.sample(rng);
                if end >= last[classes[i]] {
                    latest.push(end);
                    last[classes[i]] = end;
                    break;
                }
            }
        }

        latest
    }

    fn rng(&self) -> impl Rng {
        let init = self.seed.unwrap_or_else(|| SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis());
        let mut seed = [0_u8; 32];
        seed.iter_mut().zip(init.to_be_bytes().into_iter()).for_each(|(s, i)| *s = i);
        seed.iter_mut().rev().zip(init.to_le_bytes().into_iter()).for_each(|(s, i)| *s = i);
        ChaChaRng::from_seed(seed)
    }

}