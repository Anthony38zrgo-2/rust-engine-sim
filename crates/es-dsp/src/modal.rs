use std::f32::consts::TAU;

#[derive(Clone, Copy, Debug, Default)]
pub struct DcBlocker {
    x1: f32,
    y1: f32,
    r: f32,
}

impl DcBlocker {
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            x1: 0.0,
            y1: 0.0,
            r: (-TAU * cutoff_hz / sample_rate.max(1.0)).exp(),
        }
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = if y.abs() < 1.0e-20 { 0.0 } else { y };
        self.y1
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OnePoleLowPass {
    alpha: f32,
    state: f32,
    bypassed: bool,
}

impl OnePoleLowPass {
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        let nyquist_guard = sample_rate.max(1.0) * 0.48;
        let cutoff = cutoff_hz.clamp(1.0, nyquist_guard);
        Self {
            alpha: 1.0 - (-TAU * cutoff / sample_rate.max(1.0)).exp(),
            state: 0.0,
            bypassed: false,
        }
    }

    pub fn bypassed() -> Self {
        Self {
            alpha: 1.0,
            state: 0.0,
            bypassed: true,
        }
    }

    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        if self.bypassed {
            return input;
        }
        self.state += self.alpha * (input - self.state);
        if self.state.abs() < 1.0e-20 {
            self.state = 0.0;
        }
        self.state
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Resonator {
    a1: f32,
    a2: f32,
    gain: f32,
    y1: f32,
    y2: f32,
}

impl Resonator {
    pub fn new(frequency_hz: f32, decay_seconds: f32, gain: f32, sample_rate: f32) -> Self {
        let sr = sample_rate.max(1.0);
        let f = frequency_hz.clamp(1.0, sr * 0.48);
        let decay = decay_seconds.clamp(0.0005, 2.0);
        let radius = (-1.0 / (decay * sr)).exp();
        let omega = TAU * f / sr;
        Self {
            a1: 2.0 * radius * omega.cos(),
            a2: -(radius * radius),
            gain: gain * (1.0 - radius),
            y1: 0.0,
            y2: 0.0,
        }
    }

    #[inline]
    pub fn process(&mut self, excitation: f32) -> f32 {
        let y = excitation * self.gain + self.a1 * self.y1 + self.a2 * self.y2;
        self.y2 = self.y1;
        self.y1 = if y.abs() < 1.0e-20 { 0.0 } else { y };
        self.y1
    }
}

#[derive(Clone, Debug, Default)]
pub struct ModalBank {
    modes: Vec<Resonator>,
}

impl ModalBank {
    pub fn new(spec: &[(f32, f32, f32)], sample_rate: f32) -> Self {
        Self {
            modes: spec
                .iter()
                .map(|&(f, d, g)| Resonator::new(f, d, g, sample_rate))
                .collect(),
        }
    }

    #[inline]
    pub fn process(&mut self, excitation: f32) -> f32 {
        self.modes
            .iter_mut()
            .map(|mode| mode.process(excitation))
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resonator_decays_to_tail() {
        let mut r = Resonator::new(1_000.0, 0.05, 1.0, 48_000.0);
        let mut last = 0.0;
        for _ in 0..48_000 {
            last = r.process(0.0);
        }
        assert!(last.abs() < 1e-6, "resonator tail={last}");
    }

    #[test]
    fn resonator_responds_to_excitation() {
        let mut r = Resonator::new(1_000.0, 0.1, 1.0, 48_000.0);
        r.process(1.0);
        let mut peak = 0.0f32;
        for _ in 0..4_800 {
            peak = peak.max(r.process(0.0).abs());
        }
        assert!(peak > 1e-4, "resonator peak={peak}");
    }

    #[test]
    fn dc_blocker_removes_mean() {
        let mut dc = DcBlocker::new(20.0, 48_000.0);
        let mut last = 0.0;
        for _ in 0..48_000 {
            last = dc.process(1.0);
        }
        assert!(last.abs() < 0.01, "dc output={last}");
    }

    #[test]
    fn one_pole_bypass_is_transparent() {
        let mut lp = OnePoleLowPass::bypassed();
        assert_eq!(lp.process(0.42), 0.42);
    }

    #[test]
    fn modal_bank_sums_modes() {
        let mut bank = ModalBank::new(&[(500.0, 0.05, 1.0), (1_500.0, 0.05, 1.0)], 48_000.0);
        bank.process(1.0);
        let mut peak = 0.0f32;
        for _ in 0..4_800 {
            peak = peak.max(bank.process(0.0).abs());
        }
        assert!(peak > 1e-4, "bank peak={peak}");
    }
}
