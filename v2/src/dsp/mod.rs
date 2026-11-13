//! Allocation-free DSP building blocks: filters, modal resonators, spectrum chains,
//! and the master output safety limiter.

pub mod filters;
pub mod limiter;
pub mod resonators;
pub mod spectrum;

pub use filters::{
    BandPassNoise, Biquad, DcBlocker, OnePoleLowPass, ThreePoleLowPass, TrackingNotch,
};
pub use limiter::{
    limit_master, MASTER_LIMITER_CEILING, MASTER_LIMITER_HEADROOM, MASTER_LIMITER_KNEE,
};
pub use resonators::{ModalBank, Resonator};
pub use spectrum::{SpectrumChain, SpectrumChainConfig, UpperMidShelf};

/// Generates a deterministic Xorshift64* bipolar sample in `[-1.0, 1.0]`.
#[inline]
pub fn xorshift64_signed(rng: &mut u64) -> f32 {
    let mut x = *rng;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *rng = x;
    let bits = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40;
    bits as f32 / ((1u64 << 24) - 1) as f32 * 2.0 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f32 = 44_100.0;

    fn sine_amplitude_db(mut process: impl FnMut(f32) -> f32, frequency: f32) -> f64 {
        let total = SAMPLE_RATE as usize;
        let skip = total / 2;
        let mut sum = 0.0f64;
        for index in 0..total {
            let phase =
                2.0 * std::f64::consts::PI * frequency as f64 * index as f64 / SAMPLE_RATE as f64;
            let output = process(phase.sin() as f32);
            if index >= skip {
                sum += (output as f64) * (output as f64);
            }
        }
        let rms = (sum / (total - skip) as f64).sqrt();
        20.0 * (rms * 2.0_f64.sqrt()).log10()
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
            let phase =
                2.0 * std::f64::consts::PI * frequency as f64 * index as f64 / SAMPLE_RATE as f64;
            let output = chain.process((phase.sin() * amplitude as f64) as f32) as f64;
            if index >= skip {
                sum += output * output;
            }
        }
        (sum / (total - skip) as f64).sqrt()
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
    fn high_shelf_boosts_only_above_cutoff() {
        let mut shelf_low = Biquad::high_shelf(2_500.0, 0.707, 4.0, SAMPLE_RATE);
        let low = sine_amplitude_db(|x| shelf_low.process(x), 500.0);
        let mut shelf_high = Biquad::high_shelf(2_500.0, 0.707, 4.0, SAMPLE_RATE);
        let high = sine_amplitude_db(|x| shelf_high.process(x), 6_000.0);
        assert!(low.abs() < 0.4, "500 Hz must stay flat, got {low} dB");
        assert!(
            (high - 4.0).abs() < 1.0,
            "6 kHz must lift ~4 dB, got {high} dB"
        );
    }

    #[test]
    fn upper_mid_shelf_is_bit_transparent_at_zero_gain_and_boosts_presence() {
        let mut bypass = UpperMidShelf::new(SAMPLE_RATE, 0.0).unwrap();
        for index in 0..256 {
            let input = (index as f32 * 0.13).sin() * 0.5;
            assert_eq!(bypass.process(input), input);
        }
        assert!(UpperMidShelf::new(SAMPLE_RATE, 1.1).is_err());
        assert!(UpperMidShelf::new(SAMPLE_RATE, -0.1).is_err());

        let mut active = UpperMidShelf::new(SAMPLE_RATE, 0.5).unwrap();
        let band_db = sine_amplitude_db(|x| active.process(x), 3_200.0);
        assert!(
            band_db > 1.5,
            "presence band must boost at 3.2 kHz, got {band_db} dB"
        );
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
    fn v10_and_gearbox_chains_shape_target_bands_and_validate_bounds() {
        let v10 = v10_filter_config();
        let mid = spectrum_db(v10, 1_000.0);
        let low = spectrum_db(v10, 50.0);
        let high = spectrum_db(v10, 10_000.0);
        let top = spectrum_db(v10, 20_000.0);
        assert!(mid.abs() < 1.5);
        assert!(low < mid - 4.0);
        assert!(high > mid + 1.0);
        assert!(top < mid - 3.0);

        let gearbox = gearbox_filter_config();
        let gb_mid = spectrum_db(gearbox, 1_500.0);
        let gb_low = spectrum_db(gearbox, 200.0);
        let gb_high = spectrum_db(gearbox, 8_000.0);
        assert!(gb_mid.abs() < 2.5);
        assert!(gb_low < gb_mid - 10.0);
        assert!(gb_high < gb_mid - 10.0);

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

    #[test]
    fn peak_distortion_behaves_linearly_when_disabled_and_intermodulates_when_enabled() {
        let base = distorted_peak_config();
        let mut with_drive_off = SpectrumChain::new(
            SAMPLE_RATE,
            SpectrumChainConfig {
                peak_drive: 0.0,
                ..base
            },
        )
        .unwrap();
        let mut with_mix_off = SpectrumChain::new(
            SAMPLE_RATE,
            SpectrumChainConfig {
                peak_drive_mix: 0.0,
                ..base
            },
        )
        .unwrap();
        for index in 0..4096 {
            let phase = 2.0 * std::f64::consts::PI * 10_000.0 * index as f64 / SAMPLE_RATE as f64;
            let input = (phase.sin() * 0.5) as f32;
            assert_eq!(with_drive_off.process(input), with_mix_off.process(input));
        }

        let linear = SpectrumChainConfig {
            peak_drive: 0.0,
            ..base
        };
        let ratio = chain_rms(base, 10_000.0, 0.01) / chain_rms(linear, 10_000.0, 0.01);
        assert!((ratio - 1.0).abs() < 0.1);

        let tones = [9_500.0f32, 10_500.0f32];
        let linear_product = chain_probe_db(linear, &tones, 8_500.0);
        let distorted_product = chain_probe_db(base, &tones, 8_500.0);
        assert!(distorted_product > linear_product + 6.0);
    }

    #[test]
    fn bypassed_lowpass_three_pole_resonator_and_master_limiter_work_cleanly() {
        let mut lp = OnePoleLowPass::bypassed();
        for sample in 0..32 {
            let input = sample as f32 * 0.25 - 3.0;
            assert_eq!(lp.process(input), input);
        }

        let mut three_pole = ThreePoleLowPass::new(8_000.0, 48_000.0);
        assert!(three_pole.process(1.0).is_finite());

        let mut res = Resonator::new(400.0, 0.05, 1.0, 48_000.0);
        let peak = res.process(1.0).abs();
        for _ in 0..24_000 {
            let _ = res.process(0.0);
        }
        assert!(res.process(0.0).abs() < peak * 1.0e-3);

        let (below, red_below) = limit_master(-0.92);
        assert_eq!(below, -0.92);
        assert_eq!(red_below, 0.0);

        let (above, red_above) = limit_master(2.0);
        assert!(above > MASTER_LIMITER_KNEE && above <= MASTER_LIMITER_CEILING);
        assert!(red_above < 0.0);
    }
}
