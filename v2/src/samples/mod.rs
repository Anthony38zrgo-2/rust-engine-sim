//! Prepared sample bank playback: schema-1/2 asset loading, equal-power crossfades,
//! windowed-sinc resampling, and allocation-free `SampleBankPlayer`.

pub mod bank;
pub mod crossfade;
pub mod playback;
pub mod resampling;
pub mod zone_dsp;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::config::ThreeZoneSampleLayerConfig;

pub use bank::{PreparedMember, PreparedSampleBank, PreparedZone};
pub use crossfade::{SampleLayerInput, ZONE_COUNT};
pub use playback::{SampleBankPlayer, SampleLayerFrame};
pub use resampling::{
    build_sinc_table, sinc_sample, sinc_sample_tabled, SINC_RADIUS, SINC_TABLE_PHASES,
};

/// Backwards-compatible sample-layer facade delegating bank preparation to
/// [`PreparedSampleBank`] and real-time audio synthesis to [`SampleBankPlayer`].
#[derive(Clone, Debug)]
pub struct ThreeZoneSampleLayer {
    player: SampleBankPlayer,
}

impl ThreeZoneSampleLayer {
    /// Loads a schema-1 legacy directory or schema-2 manifest directory on the control thread.
    pub fn load_directory(
        output_sample_rate: u32,
        directory: &Path,
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        let bank = Arc::new(PreparedSampleBank::load_directory(
            output_sample_rate,
            directory,
            config,
        )?);
        Ok(Self::from_bank(bank))
    }

    /// Loads three legacy zone metadata files directly on the control thread.
    pub fn load(
        output_sample_rate: u32,
        metadata_paths: [PathBuf; ZONE_COUNT],
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        let bank = Arc::new(PreparedSampleBank::load(
            output_sample_rate,
            metadata_paths,
            config,
        )?);
        Ok(Self::from_bank(bank))
    }

    /// Wraps an already prepared immutable sample bank inside a fresh player instance.
    pub fn from_bank(bank: Arc<PreparedSampleBank>) -> Self {
        Self {
            player: SampleBankPlayer::new(bank),
        }
    }

    /// Accesses the underlying immutable bank.
    pub fn bank(&self) -> &PreparedSampleBank {
        self.player.bank()
    }

    /// Preallocates a [`SampleLayerFrame`] with full capacity for `process_into`.
    pub fn allocate_frame(&self) -> SampleLayerFrame {
        self.player.allocate_frame()
    }

    /// Aligns phase cursors to `crank_phase_deg`.
    pub fn reset_phase(&mut self, crank_phase_deg: f32) -> Result<(), String> {
        self.player.reset_phase(crank_phase_deg)
    }

    /// Returns ON zone RPM anchors as a newly allocated vector.
    pub fn rpm_anchors(&self) -> Vec<f32> {
        self.player.rpm_anchors()
    }

    /// Zero-allocation slice view of ON zone RPM anchors.
    pub fn rpm_anchors_slice(&self) -> &[f32] {
        self.player.rpm_anchors_slice()
    }

    /// Diagnostic labels aligned with `SampleLayerFrame::zone_weights`.
    pub fn source_labels(&self) -> &[String] {
        self.player.source_labels()
    }

    /// Count of OFF source loops across all OFF zones.
    pub fn off_source_count(&self) -> usize {
        self.player.off_source_count()
    }

    /// Cumulative skipped cursor-only evaluations.
    pub fn suspended_zone_samples(&self) -> u64 {
        self.player.suspended_zone_samples()
    }

    /// Convenience sample render allocating one output frame.
    #[inline]
    pub fn process(&mut self, input: SampleLayerInput) -> Result<SampleLayerFrame, String> {
        self.player.process(input)
    }

    /// Allocation-free sample render into caller-owned `out`.
    #[inline]
    pub fn process_into(
        &mut self,
        input: SampleLayerInput,
        out: &mut SampleLayerFrame,
    ) -> Result<(), String> {
        self.player.process_into(input, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::samples::crossfade::{
        legacy_zone_weights, multi_zone_weights_into, off_throttle_weight,
    };

    fn fixtures_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
    }

    #[test]
    fn crossfades_have_constant_power() {
        let anchors = [7_500.0, 8_200.0, 15_000.0];
        for rpm in [7_500.0, 8_750.0, 10_000.0, 12_500.0, 15_000.0] {
            let weights = legacy_zone_weights(rpm, anchors, 10_000.0, 13_750.0);
            let power: f32 = weights.iter().map(|weight| weight * weight).sum();
            assert!((power - 1.0).abs() < 1.0e-6);
        }
        assert_eq!(
            legacy_zone_weights(9_999.0, anchors, 10_000.0, 13_750.0)[2],
            0.0
        );
        assert_eq!(
            legacy_zone_weights(13_750.0, anchors, 10_000.0, 13_750.0),
            [0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn multi_zone_weights_into_is_equal_power_and_clamped_at_extremes() {
        let anchors = [3_800.0, 6_800.0, 12_500.0];
        let mut weights = Vec::with_capacity(anchors.len());
        for rpm in [
            2_000.0, 3_800.0, 5_300.0, 6_800.0, 9_650.0, 12_500.0, 16_000.0,
        ] {
            multi_zone_weights_into(rpm, &anchors, &mut weights);
            let power: f32 = weights.iter().map(|w| w * w).sum();
            assert!((power - 1.0).abs() < 1.0e-6);
        }
        multi_zone_weights_into(2_500.0, &anchors, &mut weights);
        assert_eq!(weights, [1.0, 0.0, 0.0]);
        multi_zone_weights_into(15_000.0, &anchors, &mut weights);
        assert_eq!(weights, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn sinc_resampler_wraps_and_tabled_matches_reference() {
        let source_short = [0.0, 1.0, 0.0, -1.0];
        for cursor in [0.0, 0.5, 3.75, 4.0, 9.25] {
            assert!(sinc_sample(&source_short, cursor).is_finite());
        }
        assert!(sinc_sample(&source_short, 0.0).abs() < 1.0e-6);
        assert!((sinc_sample(&source_short, 1.0) - 1.0).abs() < 1.0e-6);

        let source: Vec<f32> = (0..4096)
            .map(|n| {
                let t = n as f64;
                (0.5 * (t * 0.11).sin()
                    + 0.3 * (t * 0.031 + 1.0).sin()
                    + 0.2 * (t * 0.53 + 2.0).sin()) as f32
            })
            .collect();
        let table = build_sinc_table(SINC_TABLE_PHASES, SINC_RADIUS);
        let taps = (2 * SINC_RADIUS) as usize;
        let mut max_diff = 0.0f32;
        for rate in [0.35, 0.5, 0.9, 1.0, 1.25, 1.9, 2.4] {
            for direction in [1.0, -1.0] {
                let mut cursor = 123.456f64;
                for _ in 0..4_000 {
                    let reference = sinc_sample(&source, cursor);
                    let tabled = sinc_sample_tabled(
                        &source,
                        cursor,
                        &table,
                        SINC_TABLE_PHASES,
                        -SINC_RADIUS + 1,
                        taps,
                    );
                    assert!(reference.is_finite() && tabled.is_finite());
                    max_diff = max_diff.max((reference - tabled).abs());
                    cursor = (cursor + direction * rate).rem_euclid(source.len() as f64);
                }
            }
        }
        assert!(max_diff < 1.0e-4, "tabled sinc max_diff={max_diff}");
    }

    #[test]
    fn off_weight_gates_on_clutch_and_handles_retention_vs_low_load() {
        let base = SampleLayerInput {
            rpm: 8_000.0,
            throttle: 0.0,
            load: 0.8,
            normalized_engine_torque: -0.8,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        assert!((off_throttle_weight(base, 0.20) - 0.16).abs() < 1.0e-6);

        let free_rev = SampleLayerInput {
            rpm: 9_000.0,
            throttle: 0.0,
            load: 0.0,
            normalized_engine_torque: 0.0,
            clutch_engagement: 0.0,
            crank_phase_deg: 0.0,
        };
        assert_eq!(off_throttle_weight(free_rev, 0.20), 0.0);

        let coast = SampleLayerInput {
            clutch_engagement: 1.0,
            ..free_rev
        };
        assert!(off_throttle_weight(coast, 0.20) > 0.0);

        let half = SampleLayerInput {
            clutch_engagement: 0.5,
            ..coast
        };
        assert!(
            (off_throttle_weight(half, 0.20) - off_throttle_weight(coast, 0.20) * 0.5).abs()
                < 1.0e-6
        );

        let positive_low_load = SampleLayerInput {
            rpm: 8_000.0,
            throttle: 0.0,
            load: 0.2,
            normalized_engine_torque: 0.5,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        assert!((off_throttle_weight(positive_low_load, 0.20) - 0.10).abs() < 1.0e-6);
    }

    #[test]
    fn residual_scale_and_disable_rasp_affect_stems_predictably_on_fixture_banks() {
        let bank_dir = fixtures_dir().join("bank_schema2");
        let input = SampleLayerInput {
            rpm: 12_500.0,
            throttle: 0.95,
            load: 0.90,
            normalized_engine_torque: 0.90,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        let mut full = ThreeZoneSampleLayer::load_directory(
            48_000,
            &bank_dir,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        let mut half = ThreeZoneSampleLayer::load_directory(
            48_000,
            &bank_dir,
            ThreeZoneSampleLayerConfig {
                residual_gain_scale: 0.5,
                ..ThreeZoneSampleLayerConfig::default()
            },
        )
        .unwrap();
        let mut muted_rasp = ThreeZoneSampleLayer::load_directory(
            48_000,
            &bank_dir,
            ThreeZoneSampleLayerConfig {
                disable_sample_rasp: true,
                ..ThreeZoneSampleLayerConfig::default()
            },
        )
        .unwrap();

        let mut loud = SampleLayerFrame::default();
        let mut quiet = SampleLayerFrame::default();
        let mut no_rasp = SampleLayerFrame::default();
        // Warm up filters briefly so high-band rasp and residual band accumulate.
        for _ in 0..64 {
            full.process_into(input, &mut loud).unwrap();
            half.process_into(input, &mut quiet).unwrap();
            muted_rasp.process_into(input, &mut no_rasp).unwrap();
        }
        assert!((quiet.residual - loud.residual * 0.5).abs() < 1.0e-6);
        assert!((quiet.tonal - loud.tonal).abs() < 1.0e-6);
        assert!(loud.max_rasp.abs() > 1.0e-6);
        assert_eq!(no_rasp.max_rasp, 0.0);
        assert!((loud.output - no_rasp.output).abs() > 1.0e-6);
    }

    #[test]
    fn two_players_share_single_prepared_bank_bit_identically() {
        let bank = Arc::new(
            PreparedSampleBank::load_directory(
                48_000,
                &fixtures_dir().join("bank_schema2"),
                ThreeZoneSampleLayerConfig::default(),
            )
            .unwrap(),
        );
        let mut player_a = SampleBankPlayer::new(Arc::clone(&bank));
        let mut player_b = SampleBankPlayer::new(bank);
        let mut frame_a = player_a.allocate_frame();
        let mut frame_b = player_b.allocate_frame();
        for i in 0..256 {
            let input = SampleLayerInput {
                rpm: 3_800.0 + i as f32 * 30.0,
                throttle: 0.8,
                load: 0.7,
                normalized_engine_torque: 0.65,
                clutch_engagement: 1.0,
                crank_phase_deg: (i as f32 * 18.0).rem_euclid(720.0),
            };
            player_a.process_into(input, &mut frame_a).unwrap();
            player_b.process_into(input, &mut frame_b).unwrap();
            assert_eq!(frame_a.output.to_bits(), frame_b.output.to_bits());
        }
    }
}
