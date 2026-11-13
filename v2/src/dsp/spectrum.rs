//! Spectrum-shaping chains (`SpectrumChain`) and parallel upper-mid presence shelf
//! (`UpperMidShelf`). Allocation occurs solely during construction.

use std::f32::consts::{FRAC_1_SQRT_2, PI, TAU};

use crate::dsp::filters::Biquad;

/// Parallel band-pass presence lift between `LOW_EDGE_HZ` and `HIGH_EDGE_HZ`.
#[derive(Clone, Copy, Debug)]
pub struct UpperMidShelf {
    highpass: Biquad,
    lowpass: Biquad,
    gain: f32,
}

impl UpperMidShelf {
    pub const MIN_GAIN: f32 = 0.0;
    pub const MAX_GAIN: f32 = 1.0;
    pub const LOW_EDGE_HZ: f32 = 1800.0;
    pub const HIGH_EDGE_HZ: f32 = 5600.0;

    /// Constructs the presence shelf, rejecting invalid sample rates or out-of-range gains.
    pub fn new(sample_rate: f32, gain: f32) -> Result<Self, String> {
        if !sample_rate.is_finite() || sample_rate <= 0.0 {
            return Err(format!("invalid sample rate: {sample_rate}"));
        }
        if !gain.is_finite() || !(Self::MIN_GAIN..=Self::MAX_GAIN).contains(&gain) {
            return Err(format!("upper-mid shelf gain out of range: {gain}"));
        }
        let q = FRAC_1_SQRT_2;
        Ok(Self {
            highpass: Biquad::highpass(Self::LOW_EDGE_HZ, q, sample_rate),
            lowpass: Biquad::lowpass(Self::HIGH_EDGE_HZ, q, sample_rate),
            gain,
        })
    }

    /// Configured presence gain in `0.0..=1.0`.
    pub fn gain(&self) -> f32 {
        self.gain
    }

    /// Processes a single sample (`input + gain * band`).
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let band = self.lowpass.process(self.highpass.process(input));
        input + self.gain * band
    }

    /// Resets the shelf filter states without reallocating.
    pub fn reset(&mut self) {
        self.highpass.reset();
        self.lowpass.reset();
    }
}

/// Spectrum-shaping configuration for one audio section: cascaded high-pass,
/// cascaded low-pass and an optional peaking bell, applied in that order.
/// Slopes are dB/oct and quantize to 12 dB/oct biquad stages (`Q = 0.707`).
/// A zero cutoff or slope disables the block; `Default` is a bit-transparent bypass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpectrumChainConfig {
    pub highpass_hz: f32,
    pub highpass_slope_db_per_oct: f32,
    pub lowpass_hz: f32,
    pub lowpass_slope_db_per_oct: f32,
    pub peak_hz: f32,
    pub peak_gain_db: f32,
    pub peak_q: f32,
    /// Soft-saturation drive on the peaking band (0 = off).
    pub peak_drive: f32,
    /// Blend of the distorted peaking band over the linear one (0..1).
    pub peak_drive_mix: f32,
}

impl Default for SpectrumChainConfig {
    fn default() -> Self {
        Self {
            highpass_hz: 0.0,
            highpass_slope_db_per_oct: 0.0,
            lowpass_hz: 0.0,
            lowpass_slope_db_per_oct: 0.0,
            peak_hz: 0.0,
            peak_gain_db: 0.0,
            peak_q: 1.0,
            peak_drive: 0.0,
            peak_drive_mix: 0.0,
        }
    }
}

impl SpectrumChainConfig {
    /// Returns `true` when every stage is disabled.
    pub fn is_bypass(&self) -> bool {
        (self.highpass_hz <= 0.0 || self.highpass_slope_db_per_oct <= 0.0)
            && (self.lowpass_hz <= 0.0 || self.lowpass_slope_db_per_oct <= 0.0)
            && (self.peak_hz <= 0.0 || self.peak_gain_db == 0.0)
    }
}

#[derive(Clone, Copy)]
enum BlockKind {
    HighPass,
    LowPass,
}

/// 2x-oversampled symmetric soft saturation for the peaking band.
#[derive(Clone, Debug)]
struct PeakDistortion {
    even_taps: Vec<f32>,
    odd_taps: Vec<f32>,
    decimation_taps: Vec<f32>,
    input_history: Vec<f32>,
    input_cursor: usize,
    output_history: Vec<f32>,
    output_cursor: usize,
    drive: f32,
    mix: f32,
}

impl PeakDistortion {
    const HALF_BAND_LENGTH: usize = 33;
    const LATENCY: usize = Self::HALF_BAND_LENGTH / 2;

    fn new(drive: f32, mix: f32) -> Result<Self, String> {
        if !drive.is_finite() || drive <= 0.0 {
            return Err(format!("invalid peak drive: {drive}"));
        }
        if !mix.is_finite() || !(0.0..=1.0).contains(&mix) {
            return Err(format!("invalid peak distortion mix: {mix}"));
        }
        let impulse = Self::half_band_impulse();
        let even_taps: Vec<f32> = impulse.iter().step_by(2).copied().collect();
        let odd_taps: Vec<f32> = impulse.iter().skip(1).step_by(2).copied().collect();
        let input_length = even_taps.len();
        let output_length = impulse.len();
        Ok(Self {
            even_taps,
            odd_taps,
            decimation_taps: impulse,
            input_history: vec![0.0; input_length],
            input_cursor: 0,
            output_history: vec![0.0; output_length],
            output_cursor: 0,
            drive,
            mix,
        })
    }

    fn half_band_impulse() -> Vec<f32> {
        let length = Self::HALF_BAND_LENGTH;
        let center = (length - 1) as f32 / 2.0;
        let normalized_cutoff = 0.5;
        let mut impulse = Vec::with_capacity(length);
        let mut sum = 0.0;
        for index in 0..length {
            let offset = index as f32 - center;
            let phase = PI * normalized_cutoff * offset;
            let sinc = if phase.abs() < 1e-6 {
                1.0
            } else {
                phase.sin() / phase
            };
            let window = 0.54 - 0.46 * (TAU * index as f32 / (length - 1) as f32).cos();
            let tap = normalized_cutoff * sinc * window;
            impulse.push(tap);
            sum += tap;
        }
        for tap in &mut impulse {
            *tap /= sum;
        }
        impulse
    }

    fn dot_ring(taps: &[f32], history: &[f32], cursor: usize) -> f32 {
        let length = history.len();
        let mut sum = 0.0;
        let mut index = cursor % length;
        for &tap in taps {
            sum += tap * history[index];
            index = (index + length - 1) % length;
        }
        sum
    }

    fn push_output(&mut self, value: f32) -> f32 {
        let length = self.output_history.len();
        self.output_history[self.output_cursor] = value;
        let cursor = self.output_cursor;
        self.output_cursor = (self.output_cursor + 1) % length;
        Self::dot_ring(&self.decimation_taps, &self.output_history, cursor)
    }

    #[inline]
    fn process(&mut self, band: f32) -> f32 {
        let input_length = self.input_history.len();
        self.input_history[self.input_cursor] = band;
        let cursor = self.input_cursor;
        self.input_cursor = (self.input_cursor + 1) % input_length;
        let upsampled_even = 2.0 * Self::dot_ring(&self.even_taps, &self.input_history, cursor);
        let upsampled_odd = 2.0 * Self::dot_ring(&self.odd_taps, &self.input_history, cursor);
        let shaped_even = (upsampled_even * self.drive).tanh() / self.drive;
        let shaped_odd = (upsampled_odd * self.drive).tanh() / self.drive;
        let downsampled = self.push_output(shaped_even);
        let _ = self.push_output(shaped_odd);
        downsampled
    }

    fn reset(&mut self) {
        self.input_history.fill(0.0);
        self.input_cursor = 0;
        self.output_history.fill(0.0);
        self.output_cursor = 0;
    }
}

/// Fixed-length sample delay used to align the linear bell with oversampled distortion.
#[derive(Clone, Debug)]
struct DelayLine {
    samples: Vec<f32>,
    cursor: usize,
}

impl DelayLine {
    fn new(length: usize) -> Self {
        Self {
            samples: vec![0.0; length.max(1)],
            cursor: 0,
        }
    }

    #[inline]
    fn push(&mut self, value: f32) -> f32 {
        let output = self.samples[self.cursor];
        self.samples[self.cursor] = value;
        self.cursor = (self.cursor + 1) % self.samples.len();
        output
    }

    fn reset(&mut self) {
        self.samples.fill(0.0);
        self.cursor = 0;
    }
}

/// Cascaded biquad spectrum chain shared by the V10 engine and gearbox buses.
#[derive(Clone, Debug)]
pub struct SpectrumChain {
    highpass: Vec<Biquad>,
    lowpass: Vec<Biquad>,
    peak: Option<Biquad>,
    peak_distortion: Option<PeakDistortion>,
    peak_linear_delay: DelayLine,
    peak_band_delay: DelayLine,
}

impl SpectrumChain {
    /// Each biquad stage contributes 12 dB/oct, so 72 dB/oct becomes 6 stages.
    pub const STAGE_DB_PER_OCT: f32 = 12.0;
    pub const MAX_STAGES_PER_BLOCK: usize = 12;
    const CASCADE_Q: f32 = FRAC_1_SQRT_2;

    /// Constructs and allocates all filter stages and FIR delay lines up front.
    pub fn new(sample_rate: f32, config: SpectrumChainConfig) -> Result<Self, String> {
        if !sample_rate.is_finite() || sample_rate <= 0.0 {
            return Err(format!("invalid sample rate: {sample_rate}"));
        }
        let highpass = Self::build_block(
            sample_rate,
            "highpass",
            config.highpass_hz,
            config.highpass_slope_db_per_oct,
            BlockKind::HighPass,
        )?;
        let lowpass = Self::build_block(
            sample_rate,
            "lowpass",
            config.lowpass_hz,
            config.lowpass_slope_db_per_oct,
            BlockKind::LowPass,
        )?;
        let peak = if config.peak_hz > 0.0 && config.peak_gain_db != 0.0 {
            if !config.peak_hz.is_finite() || config.peak_hz >= sample_rate * 0.5 {
                return Err(format!("invalid peak cutoff: {}", config.peak_hz));
            }
            if !config.peak_q.is_finite() || config.peak_q <= 0.0 {
                return Err(format!("invalid peak q: {}", config.peak_q));
            }
            if !config.peak_gain_db.is_finite() {
                return Err(format!("invalid peak gain: {}", config.peak_gain_db));
            }
            Some(Biquad::peaking(
                config.peak_hz,
                config.peak_q,
                config.peak_gain_db,
                sample_rate,
            ))
        } else {
            None
        };
        let peak_distortion =
            if peak.is_some() && config.peak_drive > 0.0 && config.peak_drive_mix > 0.0 {
                Some(PeakDistortion::new(
                    config.peak_drive,
                    config.peak_drive_mix,
                )?)
            } else {
                None
            };
        Ok(Self {
            highpass,
            lowpass,
            peak,
            peak_distortion,
            peak_linear_delay: DelayLine::new(PeakDistortion::LATENCY),
            peak_band_delay: DelayLine::new(PeakDistortion::LATENCY),
        })
    }

    fn build_block(
        sample_rate: f32,
        label: &str,
        cutoff_hz: f32,
        slope_db_per_oct: f32,
        kind: BlockKind,
    ) -> Result<Vec<Biquad>, String> {
        if cutoff_hz <= 0.0 || slope_db_per_oct <= 0.0 {
            return Ok(Vec::new());
        }
        if !cutoff_hz.is_finite() || cutoff_hz >= sample_rate * 0.5 {
            return Err(format!("invalid {label} cutoff: {cutoff_hz}"));
        }
        if !slope_db_per_oct.is_finite() {
            return Err(format!("invalid {label} slope: {slope_db_per_oct}"));
        }
        let stages = ((slope_db_per_oct / Self::STAGE_DB_PER_OCT).round() as usize)
            .clamp(1, Self::MAX_STAGES_PER_BLOCK);
        let mut stages_out = Vec::with_capacity(stages);
        for _ in 0..stages {
            stages_out.push(match kind {
                BlockKind::HighPass => Biquad::highpass(cutoff_hz, Self::CASCADE_Q, sample_rate),
                BlockKind::LowPass => Biquad::lowpass(cutoff_hz, Self::CASCADE_Q, sample_rate),
            });
        }
        Ok(stages_out)
    }

    /// Returns `true` when the constructed chain is an empty passthrough.
    pub fn is_bypass(&self) -> bool {
        self.highpass.is_empty() && self.lowpass.is_empty() && self.peak.is_none()
    }

    /// Processes a single audio sample without heap allocations.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let mut sample = input;
        for filter in &mut self.highpass {
            sample = filter.process(sample);
        }
        for filter in &mut self.lowpass {
            sample = filter.process(sample);
        }
        let peaked = self.peak.as_mut().map(|peak| peak.process(sample));
        if let Some(peaked) = peaked {
            sample = match self.peak_distortion.as_mut() {
                Some(distortion) => {
                    let band = peaked - sample;
                    let linear = self.peak_linear_delay.push(sample);
                    let delayed_band = self.peak_band_delay.push(band);
                    linear
                        + delayed_band * (1.0 - distortion.mix)
                        + distortion.process(band) * distortion.mix
                }
                None => peaked,
            };
        }
        sample
    }

    /// Clears all internal filter states and delay lines.
    pub fn reset(&mut self) {
        for filter in &mut self.highpass {
            filter.reset();
        }
        for filter in &mut self.lowpass {
            filter.reset();
        }
        if let Some(peak) = &mut self.peak {
            peak.reset();
        }
        if let Some(distortion) = &mut self.peak_distortion {
            distortion.reset();
        }
        self.peak_linear_delay.reset();
        self.peak_band_delay.reset();
    }
}
