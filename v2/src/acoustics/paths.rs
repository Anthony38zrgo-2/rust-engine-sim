//! Per-cylinder structural propagation routes and multi-tap airborne delay path.

use crate::dsp::{DcBlocker, ModalBank, OnePoleLowPass};

/// Advances a fixed circular delay buffer by one sample without allocations.
#[inline]
pub(crate) fn advance_delay(delay: &mut [f32], cursor: &mut usize, input: f32) -> f32 {
    let arrived = delay[*cursor];
    delay[*cursor] = input;
    *cursor += 1;
    if *cursor == delay.len() {
        *cursor = 0;
    }
    arrived
}

/// Short structural propagation route per cylinder preserving cylinder identity prior to mixing.
#[derive(Clone, Debug)]
pub struct CylinderMechanicalPath {
    signature: ModalBank,
    propagation_delay: Vec<f32>,
    delay_cursor: usize,
    dc: DcBlocker,
    radiation_lowpass: OnePoleLowPass,
    polarity: f32,
}

impl CylinderMechanicalPath {
    /// Initializes cylinder mechanical modes and short propagation delay for cylinder `index`.
    pub fn new(index: usize, sample_rate: f32) -> Self {
        Self::with_layout(index, sample_rate, 5)
    }

    /// Initializes cylinder mechanical modes for bank split `bank_split` (`5` for V10, `4` for L4).
    pub fn with_layout(index: usize, sample_rate: f32, bank_split: usize) -> Self {
        let bank_split = bank_split.max(1);
        let bank = if index < bank_split { 0.0 } else { 1.0 };
        let position = (index % bank_split) as f32;
        let spread = position - (bank_split as f32 - 1.0) * 0.5;
        let base = 236.0 + spread * 11.5 + bank * 8.0;
        let delay_ms = 0.16 + position * 0.085 + bank * 0.055;
        Self {
            signature: ModalBank::new(
                &[
                    (base, 0.0095 + position * 0.00035, 0.115),
                    (base * (1.47 + bank * 0.018), 0.0072, -0.090),
                    (base * (2.08 - position * 0.012), 0.0050, 0.062),
                ],
                sample_rate,
            ),
            propagation_delay: vec![
                0.0;
                (delay_ms * 0.001 * sample_rate).round().max(1.0) as usize
            ],
            delay_cursor: 0,
            dc: DcBlocker::new(72.0 + position * 4.0, sample_rate),
            radiation_lowpass: OnePoleLowPass::new(1_350.0 + position * 85.0, sample_rate),
            polarity: if (index + index / bank_split).is_multiple_of(2) {
                1.0
            } else {
                -1.0
            },
        }
    }

    /// Propagates cylinder pressure derivative, blowdown, and runner arrival into mechanical output.
    #[inline]
    pub fn process(&mut self, derivative: f32, blowdown: f32, header: f32) -> f32 {
        let excitation = derivative * 0.72 + blowdown * 0.22 + header * 0.16;
        let arrived = advance_delay(
            &mut self.propagation_delay,
            &mut self.delay_cursor,
            excitation,
        );
        let resonant = self.signature.process(arrived * self.polarity);
        let radiated = self
            .radiation_lowpass
            .process(self.dc.process(arrived * 0.018 + resonant));
        (radiated * 10.0).tanh() / 4.0
    }
}

/// Three-tap airborne acoustic propagation delay into the cockpit listener position.
#[derive(Clone, Debug)]
pub struct AirPath {
    delay: Vec<f32>,
    cursor: usize,
    taps: [(usize, f32); 3],
}

impl AirPath {
    /// Allocates delay line for cockpit airborne reflections at `sample_rate`.
    pub fn new(sample_rate: f32) -> Self {
        let tap = |milliseconds: f32| (milliseconds * 0.001 * sample_rate).round() as usize;
        let taps = [(tap(2.11), 0.56), (tap(3.91), 0.28), (tap(6.31), 0.13)];
        let maximum = taps.iter().map(|&(delay, _)| delay).max().unwrap_or(0);
        Self {
            delay: vec![0.0; maximum + 1],
            cursor: 0,
            taps,
        }
    }

    /// Applies the 3-tap airborne reflection sum for one audio sample.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        self.delay[self.cursor] = input;
        let mut output = 0.0;
        for &(delay, gain) in &self.taps {
            let index = (self.cursor + self.delay.len() - delay) % self.delay.len();
            output += self.delay[index] * gain;
        }
        self.cursor += 1;
        if self.cursor == self.delay.len() {
            self.cursor = 0;
        }
        output
    }
}
