//! Allocation-free sample-bank player (`SampleBankPlayer`) separated from loading
//! and asset I/O. Every scratch buffer and zone cursor array is allocated once in `new`.

use std::sync::Arc;

use crate::samples::bank::{PreparedMember, PreparedSampleBank};
use crate::samples::crossfade::{
    legacy_zone_weights, multi_zone_weights_into, off_throttle_weight, variant_weights_into,
    CrossfadeScratch, SampleLayerInput, ZONE_COUNT,
};
use crate::samples::resampling::{sinc_sample, sinc_sample_tabled, SINC_RADIUS, SINC_TABLE_PHASES};
use crate::samples::zone_dsp::ZoneMidProcessor;

/// Output stems and active zone/member crossfade weights for one processed audio sample.
#[derive(Clone, Debug, Default)]
pub struct SampleLayerFrame {
    pub tonal: f32,
    pub residual: f32,
    pub max_rasp: f32,
    pub off_throttle: f32,
    pub output: f32,
    /// Per-source crossfade weights aligned with [`SampleBankPlayer::source_labels`].
    pub zone_weights: Vec<f32>,
}

impl SampleLayerFrame {
    /// Allocates a reusable frame whose `zone_weights` buffer already fits `source_count` entries.
    pub fn with_source_capacity(source_count: usize) -> Self {
        Self {
            zone_weights: Vec::with_capacity(source_count),
            ..Self::default()
        }
    }
}

#[derive(Clone, Debug)]
struct ZonePlaybackState {
    processor: ZoneMidProcessor,
    member_cursors: Vec<f64>,
}

/// Pure audio-thread player consuming an immutable [`PreparedSampleBank`].
/// Contains zero file paths or `std::fs` logic and performs zero heap allocations in `process_into`.
#[derive(Clone, Debug)]
pub struct SampleBankPlayer {
    bank: Arc<PreparedSampleBank>,
    on_states: Vec<ZonePlaybackState>,
    off_member_cursors: Vec<Vec<f64>>,
    phase_aligned: bool,
    suspended_zone_samples: u64,
    off_smoothed: f32,
    scratch: CrossfadeScratch,
}

impl SampleBankPlayer {
    /// Constructs playback states and preallocated crossfade scratch buffers from `bank`.
    pub fn new(bank: Arc<PreparedSampleBank>) -> Self {
        let sample_rate_f32 = bank.output_sample_rate as f32;
        let rasp_enabled = !bank.config.disable_sample_rasp;
        let on_states = bank
            .on_zones
            .iter()
            .map(|zone| ZonePlaybackState {
                processor: ZoneMidProcessor::new(
                    zone.processor_kind,
                    sample_rate_f32,
                    rasp_enabled,
                    zone.trim_db,
                ),
                member_cursors: vec![0.0; zone.members.len()],
            })
            .collect();
        let off_member_cursors = bank
            .off_zones
            .iter()
            .map(|zone| vec![0.0; zone.members.len()])
            .collect();
        let scratch = CrossfadeScratch::with_capacities(
            bank.on_zones.len(),
            bank.off_zones.len(),
            bank.max_variant_group_len,
        );
        Self {
            bank,
            on_states,
            off_member_cursors,
            phase_aligned: false,
            suspended_zone_samples: 0,
            off_smoothed: 0.0,
            scratch,
        }
    }

    /// Reference to the shared immutable prepared bank.
    pub fn bank(&self) -> &PreparedSampleBank {
        &self.bank
    }

    /// Allocates a [`SampleLayerFrame`] sized to avoid any reallocation during `process_into`.
    pub fn allocate_frame(&self) -> SampleLayerFrame {
        SampleLayerFrame::with_source_capacity(self.bank.source_labels.len())
    }

    /// Aligns every ON and OFF loop cursor to `crank_phase_deg` and clears off-throttle slew.
    pub fn reset_phase(&mut self, crank_phase_deg: f32) -> Result<(), String> {
        if !crank_phase_deg.is_finite() {
            return Err("sample-layer crank phase must be finite".into());
        }
        for (zone, state) in self.bank.on_zones.iter().zip(self.on_states.iter_mut()) {
            for (member, cursor) in zone.members.iter().zip(state.member_cursors.iter_mut()) {
                *cursor = member.aligned_cursor(crank_phase_deg);
            }
        }
        for (zone, cursors) in self
            .bank
            .off_zones
            .iter()
            .zip(self.off_member_cursors.iter_mut())
        {
            for (member, cursor) in zone.members.iter().zip(cursors.iter_mut()) {
                *cursor = member.aligned_cursor(crank_phase_deg);
            }
        }
        self.off_smoothed = 0.0;
        self.phase_aligned = true;
        Ok(())
    }

    /// Returns a freshly allocated `Vec` of ON zone RPM anchors (control-thread helper).
    pub fn rpm_anchors(&self) -> Vec<f32> {
        self.bank.on_anchors.clone()
    }

    /// Zero-allocation slice view of ON zone RPM anchors.
    pub fn rpm_anchors_slice(&self) -> &[f32] {
        &self.bank.on_anchors
    }

    /// Diagnostic labels aligned with `SampleLayerFrame::zone_weights`.
    pub fn source_labels(&self) -> &[String] {
        &self.bank.source_labels
    }

    /// Total number of OFF source loops across all OFF zones.
    pub fn off_source_count(&self) -> usize {
        self.bank
            .off_zones
            .iter()
            .map(|zone| zone.members.len())
            .sum()
    }

    /// Number of skipped cursor-only zone/member evaluations accumulated so far.
    pub fn suspended_zone_samples(&self) -> u64 {
        self.suspended_zone_samples
    }

    /// Resets suspended counter (diagnostic helper).
    pub fn clear_suspended_zone_samples(&mut self) {
        self.suspended_zone_samples = 0;
    }

    /// Convenience non-realtime wrapper allocating one `SampleLayerFrame`.
    #[inline]
    pub fn process(&mut self, input: SampleLayerInput) -> Result<SampleLayerFrame, String> {
        let mut frame = self.allocate_frame();
        self.process_into(input, &mut frame)?;
        Ok(frame)
    }

    /// Allocation-free sample processing writing results into `out`.
    /// Callers in real-time callbacks should pre-allocate `out` with [`Self::allocate_frame`]
    /// (or reuse the same `out` after warmup).
    #[inline]
    pub fn process_into(
        &mut self,
        input: SampleLayerInput,
        out: &mut SampleLayerFrame,
    ) -> Result<(), String> {
        let input = input.validate()?;
        if !self.phase_aligned {
            self.reset_phase(input.crank_phase_deg)?;
        }
        if self.bank.legacy_three_zone {
            return self.process_legacy_into(input, out);
        }
        self.process_multi_into(input, out)
    }

    /// Allocation-free generalized multi-zone / variant-group rendering into caller-owned `out`.
    pub fn process_multi_into(
        &mut self,
        input: SampleLayerInput,
        out: &mut SampleLayerFrame,
    ) -> Result<(), String> {
        let bank = &self.bank;
        let output_sample_rate = bank.output_sample_rate;
        let output_sample_rate_f32 = output_sample_rate as f32;
        let suspend = bank.config.suspend_inaudible_zones;
        let margin = bank.config.suspend_margin_rpm;
        let tabled = bank.config.use_tabled_sinc;

        multi_zone_weights_into(input.rpm, &bank.on_anchors, &mut self.scratch.on_weights);
        out.zone_weights.clear();
        out.zone_weights.resize(bank.source_labels.len(), 0.0);

        let mut tonal = 0.0;
        let mut residual = 0.0;
        let mut tonal_rasp = 0.0;
        let mut residual_rasp = 0.0;
        let mut weight_cursor = 0usize;

        for (index, (zone, state)) in bank
            .on_zones
            .iter()
            .zip(self.on_states.iter_mut())
            .enumerate()
        {
            let weight = self.scratch.on_weights[index];
            let (span_lo, span_hi) = bank.on_bounds[index];
            let deep_silence =
                weight == 0.0 && (input.rpm < span_lo - margin || input.rpm > span_hi + margin);
            variant_weights_into(
                input.rpm,
                zone.members.len(),
                zone.variant_span,
                &mut self.scratch.variant_weights,
            );
            if suspend && deep_silence {
                for (member, cursor) in zone.members.iter().zip(state.member_cursors.iter_mut()) {
                    advance_member(member, cursor, input.rpm, output_sample_rate);
                    self.suspended_zone_samples += 1;
                }
                weight_cursor += zone.members.len();
                continue;
            }
            let mut mixed_tonal = 0.0;
            let mut mixed_residual = 0.0;
            for ((member, cursor), &variant_weight) in zone
                .members
                .iter()
                .zip(state.member_cursors.iter_mut())
                .zip(self.scratch.variant_weights.iter())
            {
                if variant_weight == 0.0 {
                    advance_member(member, cursor, input.rpm, output_sample_rate);
                    self.suspended_zone_samples += 1;
                    out.zone_weights[weight_cursor] = 0.0;
                    weight_cursor += 1;
                    continue;
                }
                let member_weight = weight * variant_weight;
                let (member_tonal, member_residual) = render_member(
                    member,
                    cursor,
                    input.rpm,
                    output_sample_rate,
                    tabled,
                    &bank.tabled_narrow,
                );
                mixed_tonal += member_tonal * variant_weight;
                mixed_residual += member_residual * variant_weight;
                out.zone_weights[weight_cursor] = member_weight;
                weight_cursor += 1;
            }
            let (zone_tonal, zone_residual, zone_tonal_rasp, zone_residual_rasp) =
                state.processor.process(
                    mixed_tonal,
                    mixed_residual,
                    input.rpm,
                    output_sample_rate_f32,
                );
            tonal += zone_tonal * weight;
            residual += zone_residual * weight;
            tonal_rasp += zone_tonal_rasp * weight;
            residual_rasp += zone_residual_rasp * weight;
        }

        let charge = input.load * (0.35 + 0.65 * input.throttle);
        let tonal_gain = bank.config.tonal_gain_closed
            + (bank.config.tonal_gain_loaded - bank.config.tonal_gain_closed) * charge;
        let residual_gain = (bank.config.residual_gain_closed
            + (bank.config.residual_gain_loaded - bank.config.residual_gain_closed) * charge)
            * bank.config.residual_gain_scale;
        tonal *= tonal_gain;
        residual *= residual_gain;
        let max_rasp = tonal_rasp * tonal_gain + residual_rasp * residual_gain;

        let mut off_throttle_stem = 0.0;
        if !bank.off_zones.is_empty() {
            multi_zone_weights_into(
                input.rpm,
                &bank.off_anchors,
                &mut self.scratch.off_zone_weights,
            );
            let target = off_throttle_weight(input, bank.config.off_throttle_gain);
            let delta = (target - self.off_smoothed).clamp(-1.0 / 32.0, 1.0 / 32.0);
            self.off_smoothed += delta;
            let off_weight = self.off_smoothed;
            let mut mixed_tonal = 0.0;
            let mut mixed_residual = 0.0;
            for ((zone, cursors), &zone_weight) in bank
                .off_zones
                .iter()
                .zip(self.off_member_cursors.iter_mut())
                .zip(self.scratch.off_zone_weights.iter())
            {
                variant_weights_into(
                    input.rpm,
                    zone.members.len(),
                    zone.variant_span,
                    &mut self.scratch.variant_weights,
                );
                if suspend && off_weight == 0.0 {
                    for (member, cursor) in zone.members.iter().zip(cursors.iter_mut()) {
                        advance_member(member, cursor, input.rpm, output_sample_rate);
                        self.suspended_zone_samples += 1;
                    }
                    weight_cursor += zone.members.len();
                    continue;
                }
                for ((member, cursor), &variant_weight) in zone
                    .members
                    .iter()
                    .zip(cursors.iter_mut())
                    .zip(self.scratch.variant_weights.iter())
                {
                    let member_weight = off_weight * zone_weight * variant_weight;
                    if member_weight == 0.0 {
                        advance_member(member, cursor, input.rpm, output_sample_rate);
                        self.suspended_zone_samples += 1;
                        out.zone_weights[weight_cursor] = 0.0;
                        weight_cursor += 1;
                        continue;
                    }
                    let (member_tonal, member_residual) = render_member(
                        member,
                        cursor,
                        input.rpm,
                        output_sample_rate,
                        tabled,
                        &bank.tabled_narrow,
                    );
                    mixed_tonal += member_tonal * zone_weight * variant_weight;
                    mixed_residual += member_residual * zone_weight * variant_weight;
                    out.zone_weights[weight_cursor] = member_weight;
                    weight_cursor += 1;
                }
            }
            tonal += mixed_tonal * off_weight;
            residual += mixed_residual * off_weight;
            off_throttle_stem = (mixed_tonal + mixed_residual) * off_weight;
        }

        out.tonal = tonal;
        out.residual = residual;
        out.max_rasp = max_rasp;
        out.off_throttle = off_throttle_stem;
        out.output = tonal + residual;
        Ok(())
    }

    fn process_legacy_into(
        &mut self,
        input: SampleLayerInput,
        out: &mut SampleLayerFrame,
    ) -> Result<(), String> {
        let bank = &self.bank;
        if bank.on_zones.len() != ZONE_COUNT {
            return Err("legacy bank requires exactly three ON zones".into());
        }
        let mut legacy = [0.0f32; ZONE_COUNT];
        for (slot, zone) in legacy.iter_mut().zip(bank.on_zones.iter()) {
            *slot = zone.anchor;
        }
        let weights = legacy_zone_weights(
            input.rpm,
            legacy,
            bank.config.max_fade_start_rpm,
            bank.config.max_full_rpm,
        );
        let mut tonal = 0.0;
        let mut residual = 0.0;
        let mut tonal_rasp = 0.0;
        let mut residual_rasp = 0.0;
        let output_sample_rate = bank.output_sample_rate;
        let suspend = bank.config.suspend_inaudible_zones;
        let margin = bank.config.suspend_margin_rpm;
        let tabled = bank.config.use_tabled_sinc;
        let fade_start = bank.config.max_fade_start_rpm;
        let fade_full = bank.config.max_full_rpm;

        out.zone_weights.clear();
        out.zone_weights.resize(bank.source_labels.len(), 0.0);

        for (index, (zone, state)) in bank
            .on_zones
            .iter()
            .zip(self.on_states.iter_mut())
            .enumerate()
        {
            let weight = weights[index];
            let deep_silence = weight == 0.0
                && match index {
                    0 => input.rpm > legacy[1] + margin,
                    1 => {
                        input.rpm < legacy[0] - margin
                            || (input.rpm > legacy[1] + margin && input.rpm < fade_start - margin)
                            || input.rpm > fade_full + margin
                    }
                    _ => input.rpm < fade_start - margin,
                };
            let member = &zone.members[0];
            let cursor = &mut state.member_cursors[0];
            if suspend && deep_silence {
                advance_member(member, cursor, input.rpm, output_sample_rate);
                self.suspended_zone_samples += 1;
                continue;
            }
            let (zone_tonal, zone_residual) = render_member(
                member,
                cursor,
                input.rpm,
                output_sample_rate,
                tabled,
                &bank.tabled_narrow,
            );
            let (zone_tonal, zone_residual, zone_tonal_rasp, zone_residual_rasp) =
                state.processor.process(
                    zone_tonal,
                    zone_residual,
                    input.rpm,
                    output_sample_rate as f32,
                );
            tonal += zone_tonal * weight;
            residual += zone_residual * weight;
            tonal_rasp += zone_tonal_rasp * weight;
            residual_rasp += zone_residual_rasp * weight;
            out.zone_weights[index] = weight;
        }

        let charge = input.load * (0.35 + 0.65 * input.throttle);
        let tonal_gain = bank.config.tonal_gain_closed
            + (bank.config.tonal_gain_loaded - bank.config.tonal_gain_closed) * charge;
        let residual_gain = (bank.config.residual_gain_closed
            + (bank.config.residual_gain_loaded - bank.config.residual_gain_closed) * charge)
            * bank.config.residual_gain_scale;
        tonal *= tonal_gain;
        residual *= residual_gain;
        let max_rasp = tonal_rasp * tonal_gain + residual_rasp * residual_gain;

        let mut off_throttle_stem = 0.0;
        if let (Some(off_zone), Some(off_cursors)) =
            (bank.off_zones.first(), self.off_member_cursors.first_mut())
        {
            let target = off_throttle_weight(input, bank.config.off_throttle_gain);
            let delta = (target - self.off_smoothed).clamp(-1.0 / 32.0, 1.0 / 32.0);
            self.off_smoothed += delta;
            let off_weight = self.off_smoothed;
            let member = &off_zone.members[0];
            let cursor = &mut off_cursors[0];
            let (off_tonal, off_residual) = if suspend && off_weight == 0.0 {
                advance_member(member, cursor, input.rpm, output_sample_rate);
                self.suspended_zone_samples += 1;
                (0.0, 0.0)
            } else {
                render_member(
                    member,
                    cursor,
                    input.rpm,
                    output_sample_rate,
                    tabled,
                    &bank.tabled_narrow,
                )
            };
            tonal += off_tonal * off_weight;
            residual += off_residual * off_weight;
            off_throttle_stem = (off_tonal + off_residual) * off_weight;
            let off_index = bank.on_zones.len();
            if let Some(slot) = out.zone_weights.get_mut(off_index) {
                *slot = off_weight;
            }
        }

        out.tonal = tonal;
        out.residual = residual;
        out.max_rasp = max_rasp;
        out.off_throttle = off_throttle_stem;
        out.output = tonal + residual;
        Ok(())
    }
}

#[inline]
fn advance_member(member: &PreparedMember, cursor: &mut f64, rpm: f32, output_sample_rate: u32) {
    let rate = member.rate(rpm, output_sample_rate);
    *cursor = (*cursor + rate).rem_euclid(member.tonal.len() as f64);
}

#[inline]
fn render_member(
    member: &PreparedMember,
    cursor: &mut f64,
    rpm: f32,
    output_sample_rate: u32,
    tabled: bool,
    tabled_narrow: &[f32],
) -> (f32, f32) {
    if !tabled {
        let tonal = sinc_sample(&member.tonal, *cursor);
        let residual = sinc_sample(&member.residual, *cursor);
        advance_member(member, cursor, rpm, output_sample_rate);
        return (tonal, residual);
    }
    let rate = member.rate(rpm, output_sample_rate);
    let tap_lo = -SINC_RADIUS + 1;
    let taps = (2 * SINC_RADIUS) as usize;
    let tonal = sinc_sample_tabled(
        &member.tonal,
        *cursor,
        tabled_narrow,
        SINC_TABLE_PHASES,
        tap_lo,
        taps,
    );
    let residual = sinc_sample_tabled(
        &member.residual,
        *cursor,
        tabled_narrow,
        SINC_TABLE_PHASES,
        tap_lo,
        taps,
    );
    *cursor = (*cursor + rate).rem_euclid(member.tonal.len() as f64);
    (tonal, residual)
}
