//! Per-zone tonal/residual shaping processor using shared `crate::dsp` primitives.

use crate::dsp::{OnePoleLowPass, TrackingNotch};

/// Shapes one ON zone's tonal and residual stems with band emphasis, optional top-zone
/// +2.5 dB rasp, fifth-order tracking notch (`rpm / 12`), and parallel residual saturation.
#[derive(Clone, Copy, Debug)]
pub struct ZoneMidProcessor {
    tonal_low: OnePoleLowPass,
    tonal_high: OnePoleLowPass,
    residual_low: OnePoleLowPass,
    residual_high: OnePoleLowPass,
    rasp_tonal_low: OnePoleLowPass,
    rasp_tonal_high: OnePoleLowPass,
    rasp_residual_low: OnePoleLowPass,
    rasp_residual_high: OnePoleLowPass,
    tonal_gain: f32,
    residual_gain: f32,
    compressor_envelope: f32,
    compressor_attack: f32,
    compressor_release: f32,
    compressor_threshold: f32,
    compressor_parallel_mix: f32,
    saturation_drive: f32,
    saturation_mix: f32,
    order_five_wet: f32,
    rasp_gain: f32,
    zone_tonal_gain: f32,
    zone_residual_gain: f32,
    zone_trim_gain: f32,
    order_five_notch: TrackingNotch,
}

impl ZoneMidProcessor {
    /// Initializes processor filters and gains for zone index `0` (low), `1` (mid), or `2` (max).
    pub fn new(zone: usize, sample_rate: f32, rasp_enabled: bool, zone_trim_db: f32) -> Self {
        let (
            low_hz,
            high_hz,
            tonal_db,
            residual_db,
            order_five_wet,
            rasp_db,
            zone_tonal_db,
            zone_residual_db,
        ) = match zone {
            0 => (250.0, 900.0, 1.0, 4.0, 0.0, 0.0, 0.0, 3.0),
            1 => (350.0, 1_800.0, 1.5, 3.0, 0.0, 0.0, 2.5, 3.5),
            _ => (500.0, 2_500.0, 0.75, 2.5, 0.30, 2.5, 2.5, 3.0),
        };
        let rasp_db = if rasp_enabled { rasp_db } else { 0.0 };
        Self {
            tonal_low: OnePoleLowPass::new_unclamped(low_hz, sample_rate),
            tonal_high: OnePoleLowPass::new_unclamped(high_hz, sample_rate),
            residual_low: OnePoleLowPass::new_unclamped(low_hz, sample_rate),
            residual_high: OnePoleLowPass::new_unclamped(high_hz, sample_rate),
            rasp_tonal_low: OnePoleLowPass::new_unclamped(1_800.0, sample_rate),
            rasp_tonal_high: OnePoleLowPass::new_unclamped(6_500.0, sample_rate),
            rasp_residual_low: OnePoleLowPass::new_unclamped(1_800.0, sample_rate),
            rasp_residual_high: OnePoleLowPass::new_unclamped(6_500.0, sample_rate),
            tonal_gain: 10.0f32.powf(tonal_db / 20.0),
            residual_gain: 10.0f32.powf(residual_db / 20.0),
            compressor_envelope: 0.0,
            compressor_attack: 1.0 - (-1.0 / (0.025 * sample_rate)).exp(),
            compressor_release: 1.0 - (-1.0 / (0.120 * sample_rate)).exp(),
            compressor_threshold: if zone == 2 { 0.0275 } else { 0.040 },
            compressor_parallel_mix: if zone == 2 { 0.425 } else { 0.30 },
            saturation_drive: if zone == 2 { 1.7 } else { 1.4 },
            saturation_mix: if zone == 2 { 0.275 } else { 0.20 },
            order_five_wet,
            rasp_gain: 10.0f32.powf(rasp_db / 20.0),
            zone_tonal_gain: 10.0f32.powf(zone_tonal_db / 20.0),
            zone_residual_gain: 10.0f32.powf(zone_residual_db / 20.0),
            zone_trim_gain: 10.0f32.powf(zone_trim_db / 20.0),
            order_five_notch: TrackingNotch::new(),
        }
    }

    /// Processes `(tonal, residual)` returning `(tonal, residual, tonal_rasp, residual_rasp)`.
    #[inline]
    pub fn process(
        &mut self,
        tonal: f32,
        residual: f32,
        rpm: f32,
        sample_rate: f32,
    ) -> (f32, f32, f32, f32) {
        let tonal_band = self.tonal_high.process(tonal) - self.tonal_low.process(tonal);
        let residual_band =
            self.residual_high.process(residual) - self.residual_low.process(residual);
        let tonal_rasp = self.rasp_tonal_high.process(tonal) - self.rasp_tonal_low.process(tonal);
        let residual_rasp =
            self.rasp_residual_high.process(residual) - self.rasp_residual_low.process(residual);
        let tonal_shaped =
            tonal + tonal_band * (self.tonal_gain - 1.0) + tonal_rasp * (self.rasp_gain - 1.0);
        let tonal_shaped = self.order_five_notch.process(
            tonal_shaped,
            rpm / 12.0,
            sample_rate,
            self.order_five_wet,
        );

        let level = residual_band.abs();
        let envelope_rate = if level > self.compressor_envelope {
            self.compressor_attack
        } else {
            self.compressor_release
        };
        self.compressor_envelope += envelope_rate * (level - self.compressor_envelope);
        let compressed_gain = if self.compressor_envelope > self.compressor_threshold {
            (self.compressor_threshold / self.compressor_envelope).sqrt()
        } else {
            1.0
        };
        let boosted_band = residual_band * self.residual_gain;
        let parallel_band = boosted_band
            * ((1.0 - self.compressor_parallel_mix)
                + self.compressor_parallel_mix * compressed_gain * 1.25);
        let saturated_band =
            (parallel_band * self.saturation_drive).tanh() / self.saturation_drive.tanh();
        let processed_band =
            parallel_band * (1.0 - self.saturation_mix) + saturated_band * self.saturation_mix;
        let residual_shaped =
            residual - residual_band + processed_band + residual_rasp * (self.rasp_gain - 1.0);
        (
            tonal_shaped * self.zone_tonal_gain * self.zone_trim_gain,
            residual_shaped * self.zone_residual_gain * self.zone_trim_gain,
            if self.rasp_gain > 1.0 {
                tonal_rasp * self.rasp_gain * self.zone_tonal_gain * self.zone_trim_gain
            } else {
                0.0
            },
            if self.rasp_gain > 1.0 {
                residual_rasp * self.rasp_gain * self.zone_residual_gain * self.zone_trim_gain
            } else {
                0.0
            },
        )
    }
}
