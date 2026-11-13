//! Crankshaft angle integrator and 10-cylinder firing schedule.

pub use crate::config::physical::CYLINDER_COUNT;

/// Full four-stroke mechanical cycle in crank degrees.
pub const CYCLE_DEG: f64 = 720.0;

/// Per-sample firing bitmask and wrapped crankshaft phase.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FireEvents {
    pub mask: u16,
    pub crank_phase_deg: f32,
}

impl FireEvents {
    /// Returns `true` when `cylinder` fired on the stepped sample.
    #[inline]
    pub fn fired(self, cylinder: usize) -> bool {
        self.mask & (1 << cylinder) != 0
    }

    /// Number of cylinders that fired on the stepped sample.
    #[inline]
    pub fn count(self) -> u32 {
        self.mask.count_ones()
    }
}

/// Even-fire 72-degree V10 crankshaft event generator.
#[derive(Clone, Copy, Debug)]
pub struct Crankshaft {
    sample_rate: f64,
    cylinder_count: usize,
    firing_order: [usize; CYLINDER_COUNT],
    next_fire_abs: [f64; CYLINDER_COUNT],
    absolute_phase_deg: f64,
}

impl Crankshaft {
    /// Initializes crankshaft timing at `0°` using `firing_order`.
    pub fn new(sample_rate: u32, firing_order: [usize; CYLINDER_COUNT]) -> Self {
        Self::with_cylinder_count(sample_rate, firing_order, CYLINDER_COUNT)
    }

    /// Initializes even-fire four-stroke crankshaft timing across `cylinder_count` active cylinders.
    pub fn with_cylinder_count(
        sample_rate: u32,
        firing_order: [usize; CYLINDER_COUNT],
        cylinder_count: usize,
    ) -> Self {
        let cylinder_count = cylinder_count.clamp(1, CYLINDER_COUNT);
        let interval_deg = if cylinder_count == 10 {
            72.0
        } else {
            CYCLE_DEG / cylinder_count as f64
        };
        let next_fire_abs = std::array::from_fn(|slot| {
            if slot < cylinder_count {
                slot as f64 * interval_deg
            } else {
                f64::INFINITY
            }
        });
        Self {
            sample_rate: sample_rate as f64,
            cylinder_count,
            firing_order,
            next_fire_abs,
            absolute_phase_deg: 0.0,
        }
    }

    /// Advances crank phase by one audio sample at `rpm` and returns triggered cylinders.
    #[inline]
    pub fn step(&mut self, rpm: f32) -> FireEvents {
        let increment = rpm.max(0.0) as f64 * 6.0 / self.sample_rate;
        let next = self.absolute_phase_deg + increment;
        let mut events = FireEvents {
            mask: 0,
            crank_phase_deg: next.rem_euclid(CYCLE_DEG) as f32,
        };
        for slot in 0..self.cylinder_count {
            if self.next_fire_abs[slot] <= next {
                let cylinder = self.firing_order[slot];
                events.mask |= 1 << cylinder;
                self.next_fire_abs[slot] += CYCLE_DEG;
            }
        }
        self.absolute_phase_deg = next;
        events
    }

    /// Returns monotonically integrated crankshaft phase in degrees.
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

    #[test]
    fn four_cylinder_180deg_even_fire_order_1_3_4_2() {
        let order = [0, 2, 3, 1, 4, 5, 6, 7, 8, 9];
        let mut crank = Crankshaft::with_cylinder_count(48_000, order, 4);
        let mut sequence = Vec::new();
        let increment_deg = 6_000.0 * 6.0 / 48_000.0;
        while crank.absolute_phase_deg() + increment_deg < CYCLE_DEG {
            let events = crank.step(6_000.0);
            for cyl in 0..CYLINDER_COUNT {
                if events.fired(cyl) {
                    sequence.push(cyl);
                }
            }
        }
        assert_eq!(sequence, vec![0, 2, 3, 1]);

        let mut crank_rate = Crankshaft::with_cylinder_count(48_000, order, 4);
        let events: u32 = (0..(48_000 * 3))
            .map(|_| crank_rate.step(6_000.0).count())
            .sum();
        let measured_hz = events as f64 / 3.0;
        assert!((measured_hz - 6_000.0 / 30.0).abs() < 0.5, "{measured_hz}");
    }
}
