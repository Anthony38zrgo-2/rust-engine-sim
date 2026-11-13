#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn from_coefficients(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        let inverse = 1.0 / a0;
        Self {
            b0: b0 * inverse,
            b1: b1 * inverse,
            b2: b2 * inverse,
            a1: a1 * inverse,
            a2: a2 * inverse,
            z1: 0.0,
            z2: 0.0,
        }
    }

    pub(crate) fn highpass(cutoff_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        Self::from_coefficients(
            (1.0 + cos_w0) * 0.5,
            -(1.0 + cos_w0),
            (1.0 + cos_w0) * 0.5,
            1.0 + alpha,
            -2.0 * cos_w0,
            1.0 - alpha,
        )
    }

    fn lowpass(cutoff_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        Self::from_coefficients(
            (1.0 - cos_w0) * 0.5,
            1.0 - cos_w0,
            (1.0 - cos_w0) * 0.5,
            1.0 + alpha,
            -2.0 * cos_w0,
            1.0 - alpha,
        )
    }

    /// RBJ high shelf: flat below `cutoff_hz`, `gain_db` above it.
    pub(crate) fn high_shelf(cutoff_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let a = 10.0f32.powf(gain_db / 40.0);
        let w0 = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        let two_sqrt_a_alpha = 2.0 * a.sqrt() * alpha;
        Self::from_coefficients(
            a * ((a + 1.0) + (a - 1.0) * cos_w0 + two_sqrt_a_alpha),
            -2.0 * a * ((a - 1.0) + (a + 1.0) * cos_w0),
            a * ((a + 1.0) + (a - 1.0) * cos_w0 - two_sqrt_a_alpha),
            (a + 1.0) - (a - 1.0) * cos_w0 + two_sqrt_a_alpha,
            2.0 * ((a - 1.0) - (a + 1.0) * cos_w0),
            (a + 1.0) - (a - 1.0) * cos_w0 - two_sqrt_a_alpha,
        )
    }

    /// RBJ peaking EQ: `gain_db` at `cutoff_hz`, unity far from it.
    pub(crate) fn peaking(cutoff_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let frequency = cutoff_hz.clamp(1.0, sample_rate * 0.49);
        let a = 10.0f32.powf(gain_db / 40.0);
        let w0 = 2.0 * std::f32::consts::PI * frequency / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q.max(0.05));
        Self::from_coefficients(
            1.0 + alpha * a,
            -2.0 * cos_w0,
            1.0 - alpha * a,
            1.0 + alpha / a,
            -2.0 * cos_w0,
            1.0 - alpha / a,
        )
    }

    #[inline]
    pub(crate) fn process(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.z1;
        self.z1 = self.b1 * input - self.a1 * output + self.z2;
        self.z2 = self.b2 * input - self.a2 * output;
        if self.z1.abs() < 1e-30 {
            self.z1 = 0.0;
        }
        if self.z2.abs() < 1e-30 {
            self.z2 = 0.0;
        }
        if output.is_finite() {
            output
        } else {
            self.reset();
            0.0
        }
    }

    fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

#[cfg(test)]
mod biquad_tests {
    use super::*;

    const SAMPLE_RATE: f32 = 44_100.0;

    fn sine_amplitude_db(biquad: &mut Biquad, frequency: f32) -> f64 {
        let total = SAMPLE_RATE as usize;
        let skip = total / 2;
        let mut sum = 0.0f64;
        for index in 0..total {
            let phase =
                2.0 * std::f64::consts::PI * frequency as f64 * index as f64 / SAMPLE_RATE as f64;
            let output = biquad.process(phase.sin() as f32);
            if index >= skip {
                sum += (output as f64) * (output as f64);
            }
        }
        let rms = (sum / (total - skip) as f64).sqrt();
        20.0 * (rms * 2.0_f64.sqrt()).log10()
    }

    #[test]
    fn high_shelf_boosts_only_above_cutoff() {
        let mut shelf = Biquad::high_shelf(2_500.0, 0.707, 4.0, SAMPLE_RATE);
        let low = sine_amplitude_db(&mut shelf, 500.0);
        let mut shelf = Biquad::high_shelf(2_500.0, 0.707, 4.0, SAMPLE_RATE);
        let high = sine_amplitude_db(&mut shelf, 6_000.0);
        assert!(low.abs() < 0.4, "500 Hz must stay flat, got {low} dB");
        assert!((high - 4.0).abs() < 1.0, "6 kHz must lift ~4 dB, got {high} dB");
    }
}

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

    pub fn new(sample_rate: f32, gain: f32) -> Result<Self, String> {
        if !sample_rate.is_finite() || sample_rate <= 0.0 {
            return Err(format!("invalid sample rate: {sample_rate}"));
        }
        if !gain.is_finite() || !(Self::MIN_GAIN..=Self::MAX_GAIN).contains(&gain) {
            return Err(format!("upper-mid shelf gain out of range: {gain}"));
        }
        let q = std::f32::consts::FRAC_1_SQRT_2;
        Ok(Self {
            highpass: Biquad::highpass(Self::LOW_EDGE_HZ, q, sample_rate),
            lowpass: Biquad::lowpass(Self::HIGH_EDGE_HZ, q, sample_rate),
            gain,
        })
    }

    pub fn gain(&self) -> f32 {
        self.gain
    }

    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let band = self.lowpass.process(self.highpass.process(input));
        input + self.gain * band
    }

    pub fn reset(&mut self) {
        self.highpass.reset();
        self.lowpass.reset();
    }
}

/// Spectrum-shaping configuration for one audio section: cascaded high-pass,
/// cascaded low-pass and an optional peaking bell, applied in that order.
/// Slopes are dB/oct and quantize to 12 dB/oct biquad stages (Q = 0.707 per
/// stage). A zero cutoff or zero slope disables the block, so `Default` is a
/// bit-transparent bypass.
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

/// 2x-oversampled symmetric soft saturation for the peaking band. A half-band
/// FIR interpolates by zero-stuffing and decimates the shaped signal, so the odd
/// harmonics of a 10 kHz band do not fold back below Nyquist while the passband
/// stays flat enough to preserve the linear bell.
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
    /// Group delay of the interpolation+decimation pair, in input samples.
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
            let phase = std::f32::consts::PI * normalized_cutoff * offset;
            let sinc = if phase.abs() < 1e-6 {
                1.0
            } else {
                phase.sin() / phase
            };
            let window = 0.54
                - 0.46
                    * (2.0 * std::f32::consts::PI * index as f32 / (length - 1) as f32).cos();
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

/// Cascaded biquad spectrum chain shared by the V10 engine and gearbox buses.
pub struct SpectrumChain {
    highpass: Vec<Biquad>,
    lowpass: Vec<Biquad>,
    peak: Option<Biquad>,
    peak_distortion: Option<PeakDistortion>,
    peak_linear_delay: DelayLine,
    peak_band_delay: DelayLine,
}

/// Fixed-length sample delay used to time-align the linear bell with the
/// oversampled distortion path.
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

impl SpectrumChain {
    /// Each biquad stage contributes 12 dB/oct, so 72 dB/oct becomes 6 stages.
    pub const STAGE_DB_PER_OCT: f32 = 12.0;
    pub const MAX_STAGES_PER_BLOCK: usize = 12;
    const CASCADE_Q: f32 = std::f32::consts::FRAC_1_SQRT_2;

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
                BlockKind::HighPass => {
                    Biquad::highpass(cutoff_hz, Self::CASCADE_Q, sample_rate)
                }
                BlockKind::LowPass => Biquad::lowpass(cutoff_hz, Self::CASCADE_Q, sample_rate),
            });
        }
        Ok(stages_out)
    }

    pub fn is_bypass(&self) -> bool {
        self.highpass.is_empty() && self.lowpass.is_empty() && self.peak.is_none()
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f32 = 44_100.0;

    fn sine_amplitude_db(gain: f32, frequency: f32) -> f64 {
        let mut shelf = UpperMidShelf::new(SAMPLE_RATE, gain).unwrap();
        let total = SAMPLE_RATE as usize;
        let skip = total / 2;
        let mut sum = 0.0f64;
        for index in 0..total {
            let phase = 2.0 * std::f64::consts::PI * frequency as f64 * index as f64 / SAMPLE_RATE as f64;
            let output = shelf.process(phase.sin() as f32);
            if index >= skip {
                sum += (output as f64) * (output as f64);
            }
        }
        let rms = (sum / (total - skip) as f64).sqrt();
        20.0 * (rms * 2.0_f64.sqrt()).log10()
    }

    #[test]
    fn zero_gain_is_bit_transparent() {
        let mut shelf = UpperMidShelf::new(SAMPLE_RATE, 0.0).unwrap();
        let mut state = 0x1234_5678u32;
        for _ in 0..4096 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let input = (state as f32 / u32::MAX as f32) * 2.0 - 1.0;
            assert_eq!(shelf.process(input), input);
        }
    }

    #[test]
    fn boosts_band_without_heavy_top_octave_lift() {
        let center = sine_amplitude_db(0.25, 3200.0);
        let top = sine_amplitude_db(0.25, 8000.0);
        let low = sine_amplitude_db(0.25, 500.0);
        assert!(center > 1.0 && center < 2.2, "center gain {center} dB");
        assert!(top < 0.7, "8 kHz gain {top} dB");
        assert!(low.abs() < 0.3, "500 Hz gain {low} dB");
    }

    #[test]
    fn rejects_out_of_range_gain() {
        assert!(UpperMidShelf::new(SAMPLE_RATE, 1.1).is_err());
        assert!(UpperMidShelf::new(SAMPLE_RATE, -0.1).is_err());
    }

    fn spectrum_db(config: SpectrumChainConfig, frequency: f32) -> f64 {
        let mut chain = SpectrumChain::new(SAMPLE_RATE, config).unwrap();
        let total = SAMPLE_RATE as usize * 2;
        let skip = total / 2;
        let mut sum = 0.0f64;
        for index in 0..total {
            let phase =
                2.0 * std::f64::consts::PI * frequency as f64 * index as f64 / SAMPLE_RATE as f64;
            let output = chain.process(phase.sin() as f32);
            if index >= skip {
                sum += (output as f64) * (output as f64);
            }
        }
        let rms = (sum / (total - skip) as f64).sqrt();
        20.0 * (rms * 2.0_f64.sqrt()).log10()
    }

    fn v10_filter_config() -> SpectrumChainConfig {
        SpectrumChainConfig {
            highpass_hz: 97.0,
            highpass_slope_db_per_oct: 24.0,
            lowpass_hz: 13_100.0,
            lowpass_slope_db_per_oct: 72.0,
            peak_hz: 10_000.0,
            peak_gain_db: 6.0,
            peak_q: 3.0,
            ..SpectrumChainConfig::default()
        }
    }

    fn distorted_peak_config() -> SpectrumChainConfig {
        SpectrumChainConfig {
            highpass_hz: 97.0,
            highpass_slope_db_per_oct: 24.0,
            lowpass_hz: 14_410.0,
            lowpass_slope_db_per_oct: 72.0,
            peak_hz: 10_000.0,
            peak_gain_db: 3.0,
            peak_q: 3.0,
            peak_drive: 2.0,
            peak_drive_mix: 0.3,
        }
    }

    fn gearbox_filter_config() -> SpectrumChainConfig {
        SpectrumChainConfig {
            highpass_hz: 515.0,
            highpass_slope_db_per_oct: 72.0,
            lowpass_hz: 3_100.0,
            lowpass_slope_db_per_oct: 72.0,
            ..SpectrumChainConfig::default()
        }
    }

    #[test]
    fn default_spectrum_chain_is_bit_transparent() {
        let config = SpectrumChainConfig::default();
        assert!(config.is_bypass());
        let mut chain = SpectrumChain::new(SAMPLE_RATE, config).unwrap();
        assert!(chain.is_bypass());
        let mut state = 0x89ab_cdefu32;
        for _ in 0..4096 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let input = (state as f32 / u32::MAX as f32) * 2.0 - 1.0;
            assert_eq!(chain.process(input), input);
        }
    }

    #[test]
    fn v10_chain_highpasses_lowpasses_and_peaks_the_mid_high_band() {
        let config = v10_filter_config();
        let mid = spectrum_db(config, 1_000.0);
        let low = spectrum_db(config, 50.0);
        let high = spectrum_db(config, 10_000.0);
        let top = spectrum_db(config, 20_000.0);
        assert!(mid.abs() < 1.5, "1 kHz must stay near unity, got {mid} dB");
        assert!(low < mid - 4.0, "50 Hz must be high-passed, got {low} dB");
        assert!(
            high > mid + 1.0,
            "10 kHz peaking gain must lift above the mid band, got {high} dB vs {mid} dB"
        );
        assert!(top < mid - 3.0, "20 kHz must be low-passed, got {top} dB");
    }

    #[test]
    fn gearbox_chain_keeps_only_the_mesh_band() {
        let config = gearbox_filter_config();
        let mid = spectrum_db(config, 1_500.0);
        let low = spectrum_db(config, 200.0);
        let high = spectrum_db(config, 8_000.0);
        assert!(mid.abs() < 2.5, "1.5 kHz must pass, got {mid} dB");
        assert!(low < mid - 10.0, "200 Hz must be high-passed, got {low} dB");
        assert!(high < mid - 10.0, "8 kHz must be low-passed, got {high} dB");
    }

    #[test]
    fn spectrum_chain_rejects_out_of_range_cutoffs() {
        let above_nyquist = SpectrumChainConfig {
            lowpass_hz: 30_000.0,
            lowpass_slope_db_per_oct: 24.0,
            ..SpectrumChainConfig::default()
        };
        assert!(SpectrumChain::new(SAMPLE_RATE, above_nyquist).is_err());
        let bad_peak = SpectrumChainConfig {
            peak_hz: 10_000.0,
            peak_gain_db: 6.0,
            peak_q: 0.0,
            ..SpectrumChainConfig::default()
        };
        assert!(SpectrumChain::new(SAMPLE_RATE, bad_peak).is_err());
    }

    fn chain_probe_db(config: SpectrumChainConfig, inputs: &[f32], probe_hz: f32) -> f64 {
        let mut chain = SpectrumChain::new(SAMPLE_RATE, config).unwrap();
        let total = SAMPLE_RATE as usize * 2;
        let skip = total / 2;
        let mut real = 0.0f64;
        let mut imaginary = 0.0f64;
        let mut count = 0u64;
        for index in 0..total {
            let mut input = 0.0f64;
            for &frequency in inputs {
                input += (2.0 * std::f64::consts::PI * frequency as f64 * index as f64
                    / SAMPLE_RATE as f64)
                    .sin();
            }
            let output = chain.process(input as f32) as f64;
            if index >= skip {
                let phase = 2.0 * std::f64::consts::PI * probe_hz as f64 * index as f64
                    / SAMPLE_RATE as f64;
                real += output * phase.cos();
                imaginary += output * phase.sin();
                count += 1;
            }
        }
        let magnitude = (real * real + imaginary * imaginary).sqrt() / count as f64;
        20.0 * magnitude.max(1e-12).log10()
    }

    fn chain_rms(config: SpectrumChainConfig, frequency: f32, amplitude: f32) -> f64 {
        let mut chain = SpectrumChain::new(SAMPLE_RATE, config).unwrap();
        let total = SAMPLE_RATE as usize * 2;
        let skip = total / 2;
        let mut sum = 0.0f64;
        for index in 0..total {
            let phase = 2.0 * std::f64::consts::PI * frequency as f64 * index as f64
                / SAMPLE_RATE as f64;
            let output = chain.process((phase.sin() * amplitude as f64) as f32) as f64;
            if index >= skip {
                sum += output * output;
            }
        }
        (sum / (total - skip) as f64).sqrt()
    }

    #[test]
    fn peak_distortion_is_off_when_drive_or_mix_is_zero() {
        let base = distorted_peak_config();
        let drive_off = SpectrumChainConfig {
            peak_drive: 0.0,
            ..base
        };
        let mix_off = SpectrumChainConfig {
            peak_drive_mix: 0.0,
            ..base
        };
        let mut with_drive_off = SpectrumChain::new(SAMPLE_RATE, drive_off).unwrap();
        let mut with_mix_off = SpectrumChain::new(SAMPLE_RATE, mix_off).unwrap();
        for index in 0..4096 {
            let phase = 2.0 * std::f64::consts::PI * 10_000.0 * index as f64 / SAMPLE_RATE as f64;
            let input = (phase.sin() * 0.5) as f32;
            assert_eq!(with_drive_off.process(input), with_mix_off.process(input));
        }
    }

    #[test]
    fn peak_distortion_preserves_small_signals() {
        let linear = SpectrumChainConfig {
            peak_drive: 0.0,
            ..distorted_peak_config()
        };
        let linear_rms = chain_rms(linear, 10_000.0, 0.01);
        let distorted_rms = chain_rms(distorted_peak_config(), 10_000.0, 0.01);
        let ratio = distorted_rms / linear_rms;
        assert!((ratio - 1.0).abs() < 0.1, "small-signal gain drifted: {ratio}");
    }

    #[test]
    fn peak_distortion_adds_intermodulation_in_the_band() {
        let tones = [9_500.0f32, 10_500.0f32];
        let linear = SpectrumChainConfig {
            peak_drive: 0.0,
            ..distorted_peak_config()
        };
        let linear_product = chain_probe_db(linear, &tones, 8_500.0);
        let distorted_product = chain_probe_db(distorted_peak_config(), &tones, 8_500.0);
        assert!(
            distorted_product > linear_product + 6.0,
            "third-order product must grow: linear {linear_product} dB, distorted {distorted_product} dB"
        );
    }
}
