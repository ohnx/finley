//! Run statistics. This is the scoreboard: comparing two weight profiles is
//! only meaningful against these numbers on an identical job stream.

#[derive(Clone, Debug, Default)]
pub struct Metrics {
    pub ticks: u64,
    pub lots_created: u64,
    pub lots_completed: u64,
    /// Cycle time (creation -> done) per completed lot, in ticks.
    pub cycle_times: Vec<u64>,
    /// Ticks each lot spent waiting at a port for collection.
    pub delivery_waits: Vec<u64>,
    pub vehicle_busy_ticks: u64,
    pub vehicle_tick_capacity: u64,
    /// Per machine: ticks with nothing to work on.
    pub machine_idle_ticks: Vec<u64>,
    pub machine_names: Vec<String>,
    pub deadlock_events: u64,
    pub stuck_vehicle_events: u64,
    pub cycles_rotated: u64,
    /// Sampled every tick: jobs created but not yet assigned.
    pub backlog_samples: Vec<usize>,
}

fn percentile(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx]
}

impl Metrics {
    pub fn throughput_per_1k_ticks(&self) -> f64 {
        if self.ticks == 0 {
            return 0.0;
        }
        self.lots_completed as f64 * 1000.0 / self.ticks as f64
    }

    pub fn utilisation(&self) -> f64 {
        if self.vehicle_tick_capacity == 0 {
            return 0.0;
        }
        self.vehicle_busy_ticks as f64 / self.vehicle_tick_capacity as f64
    }

    pub fn mean_cycle_time(&self) -> f64 {
        if self.cycle_times.is_empty() {
            return 0.0;
        }
        self.cycle_times.iter().sum::<u64>() as f64 / self.cycle_times.len() as f64
    }

    /// The tail matters more than the mean; a policy that fixes the average by
    /// starving a few lots is a bad policy.
    pub fn p95_cycle_time(&self) -> u64 {
        let mut s = self.cycle_times.clone();
        s.sort_unstable();
        percentile(&s, 0.95)
    }

    pub fn mean_backlog(&self) -> f64 {
        if self.backlog_samples.is_empty() {
            return 0.0;
        }
        self.backlog_samples.iter().sum::<usize>() as f64 / self.backlog_samples.len() as f64
    }

    pub fn report(&self) -> String {
        let mut s = String::new();
        s.push_str(&format!("ticks                {}\n", self.ticks));
        s.push_str(&format!("lots created         {}\n", self.lots_created));
        s.push_str(&format!("lots completed       {}\n", self.lots_completed));
        s.push_str(&format!(
            "throughput           {:.2} lots / 1000 ticks\n",
            self.throughput_per_1k_ticks()
        ));
        s.push_str(&format!(
            "cycle time  mean     {:.1} ticks\n",
            self.mean_cycle_time()
        ));
        s.push_str(&format!(
            "cycle time  p95      {} ticks\n",
            self.p95_cycle_time()
        ));
        s.push_str(&format!(
            "vehicle utilisation  {:.1}%\n",
            self.utilisation() * 100.0
        ));
        s.push_str(&format!("mean backlog         {:.2} jobs\n", self.mean_backlog()));
        s.push_str(&format!("cycles rotated       {}\n", self.cycles_rotated));
        s.push_str(&format!("deadlock events      {}\n", self.deadlock_events));
        s.push_str(&format!("stuck vehicles       {}\n", self.stuck_vehicle_events));

        if !self.machine_idle_ticks.is_empty() && self.ticks > 0 {
            s.push_str("\nmachine starvation\n");
            let mut rows: Vec<(f64, &str)> = self
                .machine_idle_ticks
                .iter()
                .enumerate()
                .map(|(i, &t)| {
                    let name = self
                        .machine_names
                        .get(i)
                        .map(|s| s.as_str())
                        .unwrap_or("?");
                    (t as f64 / self.ticks as f64 * 100.0, name)
                })
                .collect();
            rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
            for (pct, name) in rows.iter().take(8) {
                s.push_str(&format!("  {:<16} {:>5.1}% idle\n", name, pct));
            }
        }
        s
    }
}
