pub const CYCLE_DEG: f64 = 720.0;
pub const CYLINDER_COUNT: usize = 10;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FireEvents {
    pub mask: u16,
    pub crank_phase_deg: f32,
}

impl FireEvents {
    #[inline]
    pub fn fired(self, cylinder: usize) -> bool {
        self.mask & (1 << cylinder) != 0
    }

    #[inline]
    pub fn count(self) -> u32 {
        self.mask.count_ones()
    }
}

pub struct Crankshaft {
    sample_rate: f64,
    firing_order: [usize; CYLINDER_COUNT],
    next_fire_abs: [f64; CYLINDER_COUNT],
    absolute_phase_deg: f64,
}

impl Crankshaft {
    pub fn new(sample_rate: u32, firing_order: [usize; CYLINDER_COUNT]) -> Self {
        let next_fire_abs = std::array::from_fn(|slot| slot as f64 * 72.0);
        Self {
            sample_rate: sample_rate as f64,
            firing_order,
            next_fire_abs,
            absolute_phase_deg: 0.0,
        }
    }

    #[inline]
    pub fn step(&mut self, rpm: f32) -> FireEvents {
        let increment = rpm.max(0.0) as f64 * 6.0 / self.sample_rate;
        let next = self.absolute_phase_deg + increment;
        let mut events = FireEvents {
            mask: 0,
            crank_phase_deg: next.rem_euclid(CYCLE_DEG) as f32,
        };
        for slot in 0..CYLINDER_COUNT {
            if self.next_fire_abs[slot] <= next {
                let cylinder = self.firing_order[slot];
                events.mask |= 1 << cylinder;
                self.next_fire_abs[slot] += CYCLE_DEG;
            }
        }
        self.absolute_phase_deg = next;
        events
    }

    pub fn absolute_phase_deg(&self) -> f64 {
        self.absolute_phase_deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ten_explicit_events_per_720_degree_cycle() {
        let order = [0, 5, 1, 6, 2, 7, 3, 8, 4, 9];
        let mut crank = Crankshaft::new(48_000, order);
        let mut count = 0;
        let increment_deg = 6_000.0 * 6.0 / 48_000.0;
        // Half-open mechanical cycle [0°, 720°): the event at exactly 720° is
        // cylinder 0 of the next cycle and must not be counted twice.
        while crank.absolute_phase_deg() + increment_deg < CYCLE_DEG {
            count += crank.step(6_000.0).count();
        }
        assert_eq!(count, 10);
    }

    #[test]
    fn firing_rate_is_rpm_over_twelve() {
        let order = [0, 5, 1, 6, 2, 7, 3, 8, 4, 9];
        let mut crank = Crankshaft::new(48_000, order);
        let samples = 48_000 * 3;
        let events: u32 = (0..samples).map(|_| crank.step(5_000.0).count()).sum();
        let measured_hz = events as f64 / 3.0;
        assert!((measured_hz - 5_000.0 / 12.0).abs() < 0.5, "{measured_hz}");
    }
}
