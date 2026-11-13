use serde::Deserialize;
use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};

const ZONE_COUNT: usize = 3;
const SINC_RADIUS: isize = 4;
/// AUD-07: fractional-phase resolution of the precomputed sinc tables.
const SINC_TABLE_PHASES: usize = 1024;

#[derive(Clone, Copy, Debug)]
pub struct SampleLayerInput {
    pub rpm: f32,
    pub throttle: f32,
    pub load: f32,
    /// Signed engine torque normalized to the physical maximum.
    pub normalized_engine_torque: f32,
    /// Physical clutch engagement; used to gate retention-only off audio.
    pub clutch_engagement: f32,
    pub crank_phase_deg: f32,
}

impl SampleLayerInput {
    fn validate(self) -> Result<Self, String> {
        if !self.rpm.is_finite() || !(0.0..=25_000.0).contains(&self.rpm) {
            return Err(format!("sample-layer rpm out of range: {}", self.rpm));
        }
        if !self.throttle.is_finite() || !(0.0..=1.0).contains(&self.throttle) {
            return Err(format!(
                "sample-layer throttle out of range: {}",
                self.throttle
            ));
        }
        if !self.load.is_finite() || !(0.0..=1.0).contains(&self.load) {
            return Err(format!("sample-layer load out of range: {}", self.load));
        }
        if !self.normalized_engine_torque.is_finite()
            || !(-1.0..=1.0).contains(&self.normalized_engine_torque)
        {
            return Err(format!(
                "sample-layer normalized torque out of range: {}",
                self.normalized_engine_torque
            ));
        }
        if !self.clutch_engagement.is_finite()
            || !(0.0..=1.0).contains(&self.clutch_engagement)
        {
            return Err(format!(
                "sample-layer clutch engagement out of range: {}",
                self.clutch_engagement
            ));
        }
        if !self.crank_phase_deg.is_finite() {
            return Err("sample-layer crank phase must be finite".into());
        }
        Ok(self)
    }
}

/// Per-zone trim slots. The schema-2 sampled bank carries nine ON zones; the
/// array is sized with headroom and legacy banks simply leave the tail zero.
pub const ZONE_TRIM_SLOTS: usize = 16;

#[derive(Clone, Copy, Debug)]
pub struct ThreeZoneSampleLayerConfig {
    pub tonal_gain_closed: f32,
    pub tonal_gain_loaded: f32,
    pub residual_gain_closed: f32,
    pub residual_gain_loaded: f32,
    pub max_fade_start_rpm: f32,
    pub max_full_rpm: f32,
    /// Physical simulation blend weight (default 1.0).
    pub physical_blend_weight: f32,
    /// Sample layer blend weight (default 1.0).
    pub sample_blend_weight: f32,
    /// Off-throttle overrun layer blend gain (default 0.20).
    pub off_throttle_gain: f32,
    /// AUD-06: skip sinc resampling + mid DSP for zones whose crossfade weight
    /// is exactly zero (or the off stem when its weight is zero), while still
    /// advancing their cursors so phase continuity is preserved. Zones with any
    /// nonzero weight — including fade edges — always render (default true).
    pub suspend_inaudible_zones: bool,
    /// AUD-06 (fix.txt): deep-silence margin in RPM. A zero-weight zone is
    /// only suspended when the RPM lies this far outside every crossfade it
    /// takes part in; zones approaching a fade keep rendering ("cercanas a un
    /// crossfade disponibles") so their filters track and are converged at
    /// fade entry. The stateless off stem needs no margin (default 500.0).
    pub suspend_margin_rpm: f32,
    /// AUD-07a (cost): resample through the precomputed windowed-sinc table
    /// instead of evaluating sin/cos/division per tap per sample. The table
    /// bakes the same radius-4 Hann kernel including per-phase normalization,
    /// so it is a transparent accelerator, not an antialias fix (default true).
    pub use_tabled_sinc: bool,
    /// Diagnostic mute: disables the top-zone rasp boost (+2.5 dB baked into
    /// the shaped tonal/residual plus the max_rasp stem). Rasp terms go to
    /// zero and the stem reports 0. Default false preserves behavior.
    pub disable_sample_rasp: bool,
    /// Diagnostic trim: linear multiplier on the load-blended residual gain
    /// (default 1.0). Lets an A/B test how much of the character is loop
    /// hiss vs tone without touching the prepared stems.
    pub residual_gain_scale: f32,
    /// Per-zone tonal trim in dB, indexed by ON zone order (low anchor first).
    /// All zeros preserves shipped behavior; used to flatten the procedural
    /// vs sample balance across RPM.
    pub zone_trim_db: [f32; ZONE_TRIM_SLOTS],
}

impl Default for ThreeZoneSampleLayerConfig {
    fn default() -> Self {
        Self {
            tonal_gain_closed: 0.05,
            tonal_gain_loaded: 0.22,
            residual_gain_closed: 0.12,
            residual_gain_loaded: 0.40,
            max_fade_start_rpm: 5_970.0,
            max_full_rpm: 8_725.0,
            physical_blend_weight: 1.0,
            sample_blend_weight: 1.0,
            off_throttle_gain: 0.20,
            suspend_inaudible_zones: true,
            suspend_margin_rpm: 500.0,
            use_tabled_sinc: true,
            disable_sample_rasp: false,
            residual_gain_scale: 1.0,
            zone_trim_db: [0.0; ZONE_TRIM_SLOTS],
        }
    }
}

impl ThreeZoneSampleLayerConfig {
    pub fn validate(self) -> Result<Self, String> {
        for (name, value) in [
            ("tonal_gain_closed", self.tonal_gain_closed),
            ("tonal_gain_loaded", self.tonal_gain_loaded),
            ("residual_gain_closed", self.residual_gain_closed),
            ("residual_gain_loaded", self.residual_gain_loaded),
            ("physical_blend_weight", self.physical_blend_weight),
            ("sample_blend_weight", self.sample_blend_weight),
            ("off_throttle_gain", self.off_throttle_gain),
        ] {
            if !value.is_finite() || !(0.0..=2.0).contains(&value) {
                return Err(format!("{name} outside 0..2.0: {value}"));
            }
        }
        if !self.suspend_margin_rpm.is_finite() || !(0.0..=5_000.0).contains(&self.suspend_margin_rpm) {
            return Err(format!("suspend_margin_rpm outside 0..5000: {}", self.suspend_margin_rpm));
        }
        if !self.residual_gain_scale.is_finite() || !(0.0..=2.0).contains(&self.residual_gain_scale)
        {
            return Err(format!(
                "residual_gain_scale outside 0..2.0: {}",
                self.residual_gain_scale
            ));
        }
        for (index, trim) in self.zone_trim_db.iter().enumerate() {
            if !trim.is_finite() || !(-12.0..=12.0).contains(trim) {
                return Err(format!("zone_trim_db[{index}] outside -12..12 dB: {trim}"));
            }
        }
        if !self.max_fade_start_rpm.is_finite()
            || !self.max_full_rpm.is_finite()
            || self.max_fade_start_rpm < 1_000.0
            || self.max_full_rpm > 25_000.0
            || self.max_full_rpm - self.max_fade_start_rpm < 500.0
        {
            return Err("invalid max-zone RPM crossfade window".into());
        }
        Ok(self)
    }
}

#[derive(Clone, Debug, Default)]
pub struct SampleLayerFrame {
    pub tonal: f32,
    pub residual: f32,
    pub max_rasp: f32,
    pub off_throttle: f32,
    pub output: f32,
    /// Per-source crossfade weights, aligned with `source_labels()`.
    /// ON sources first (zone weight x variant weight), then OFF sources
    /// (retention weight x variant weight). Diagnostic; the audio path uses
    /// the same values through the local computation.
    pub zone_weights: Vec<f32>,
}

struct OnePoleLowPass {
    alpha: f32,
    state: f32,
}

impl OnePoleLowPass {
    fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            alpha: 1.0 - (-std::f32::consts::TAU * cutoff_hz / sample_rate).exp(),
            state: 0.0,
        }
    }

    #[inline]
    fn process(&mut self, input: f32) -> f32 {
        self.state += self.alpha * (input - self.state);
        self.state
    }
}

struct TrackingNotch {
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl TrackingNotch {
    fn new() -> Self {
        Self {
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    #[inline]
    fn process(&mut self, input: f32, frequency_hz: f32, sample_rate: f32, wet: f32) -> f32 {
        if wet <= 0.0 || frequency_hz >= sample_rate * 0.45 {
            return input;
        }
        let omega = std::f32::consts::TAU * frequency_hz / sample_rate;
        let alpha = omega.sin() / (2.0 * 10.0);
        let inverse_a0 = 1.0 / (1.0 + alpha);
        let b0 = inverse_a0;
        let b1 = -2.0 * omega.cos() * inverse_a0;
        let b2 = inverse_a0;
        let a1 = b1;
        let a2 = (1.0 - alpha) * inverse_a0;
        let filtered = b0 * input + b1 * self.x1 + b2 * self.x2 - a1 * self.y1 - a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = filtered;
        input + (filtered - input) * wet
    }
}

struct ZoneMidProcessor {
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
    fn new(zone: usize, sample_rate: f32, rasp_enabled: bool, zone_trim_db: f32) -> Self {
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
            tonal_low: OnePoleLowPass::new(low_hz, sample_rate),
            tonal_high: OnePoleLowPass::new(high_hz, sample_rate),
            residual_low: OnePoleLowPass::new(low_hz, sample_rate),
            residual_high: OnePoleLowPass::new(high_hz, sample_rate),
            rasp_tonal_low: OnePoleLowPass::new(1_800.0, sample_rate),
            rasp_tonal_high: OnePoleLowPass::new(6_500.0, sample_rate),
            rasp_residual_low: OnePoleLowPass::new(1_800.0, sample_rate),
            rasp_residual_high: OnePoleLowPass::new(6_500.0, sample_rate),
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

    #[inline]
    fn process(
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

#[derive(Deserialize)]
struct PreparedFiles {
    loop_tonal: String,
    loop_residual: String,
}

#[derive(Deserialize)]
struct PreparedMetadata {
    schema_version: u32,
    sample_rate: u32,
    rpm_anchor: f32,
    firing_phase_degrees_at_loop_start: f32,
    loop_length_samples: usize,
    files: PreparedFiles,
}

/// Schema-2 experimental manifest (`manifest.json` with
/// `"schema_version": 2`, `"key": "v10_f2002_experimental"`). Roles come
/// from this manifest, never from filename suffixes.
#[derive(Deserialize)]
struct ManifestVariant {
    group: String,
    position: usize,
    count: usize,
}

#[derive(Deserialize)]
struct ManifestSample {
    key: String,
    role: String,
    metadata: String,
    native_rpm: f32,
    sample_rate: u32,
    channels: u16,
    loop_frames: usize,
    tonal: String,
    tonal_sha256: String,
    residual: String,
    residual_sha256: String,
    variant: Option<ManifestVariant>,
}

#[derive(Deserialize)]
struct ManifestSchema2 {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<ManifestSample>,
    off_samples: Vec<ManifestSample>,
}

struct ZoneMember {
    key: String,
    zone: SampleZone,
    variant_group: Option<String>,
    variant_position: usize,
    variant_total: usize,
}

struct OnZone {
    /// Mean of member anchors; defines crossfade geometry. Each member
    /// keeps its own anchor for its playback rate.
    anchor: f32,
    members: Vec<ZoneMember>,
    /// Zone DSP. Meaningful for ON zones only; OFF zones render dry and
    /// this processor is never used.
    processor: ZoneMidProcessor,
    /// Variant-interpolation span for grouped members
    /// ([lo, hi] in RPM, equal-power). Unused for single members.
    variant_span: (f32, f32),
}

struct SampleZone {
    rpm_anchor: f32,
    firing_phase_deg: f32,
    source_sample_rate: u32,
    tonal: Vec<f32>,
    residual: Vec<f32>,
    cursor: f64,
}

impl SampleZone {
    fn load(metadata_path: &Path) -> Result<Self, String> {
        let raw = fs::read_to_string(metadata_path)
            .map_err(|error| format!("cannot read {}: {error}", metadata_path.display()))?;
        let metadata: PreparedMetadata = serde_json::from_str(&raw)
            .map_err(|error| format!("invalid {}: {error}", metadata_path.display()))?;
        if metadata.schema_version != 1 {
            return Err(format!(
                "unsupported sample-layer schema {} in {}",
                metadata.schema_version,
                metadata_path.display()
            ));
        }
        if !(8_000..=192_000).contains(&metadata.sample_rate) {
            return Err(format!(
                "sample-layer source rate out of range in {}: {}",
                metadata_path.display(),
                metadata.sample_rate
            ));
        }
        if !metadata.rpm_anchor.is_finite() || !(1_000.0..=25_000.0).contains(&metadata.rpm_anchor)
        {
            return Err(format!("invalid RPM anchor in {}", metadata_path.display()));
        }
        if !metadata.firing_phase_degrees_at_loop_start.is_finite() {
            return Err(format!(
                "invalid firing phase in {}",
                metadata_path.display()
            ));
        }
        let parent = metadata_path.parent().unwrap_or_else(|| Path::new("."));
        let tonal = read_mono_pcm16(&parent.join(&metadata.files.loop_tonal))?;
        let residual = read_mono_pcm16(&parent.join(&metadata.files.loop_residual))?;
        if tonal.sample_rate != metadata.sample_rate || residual.sample_rate != metadata.sample_rate
        {
            return Err(format!(
                "WAV/metadata sample-rate mismatch in {}",
                metadata_path.display()
            ));
        }
        if tonal.samples.len() != metadata.loop_length_samples
            || residual.samples.len() != metadata.loop_length_samples
            || tonal.samples.len() < 32
        {
            return Err(format!(
                "WAV/metadata loop-length mismatch in {}",
                metadata_path.display()
            ));
        }
        Ok(Self {
            rpm_anchor: metadata.rpm_anchor,
            firing_phase_deg: metadata
                .firing_phase_degrees_at_loop_start
                .rem_euclid(360.0),
            source_sample_rate: metadata.sample_rate,
            tonal: tonal.samples,
            residual: residual.samples,
            cursor: 0.0,
        })
    }

    fn align(&mut self, crank_phase_deg: f32) {
        let target_firing_phase = (crank_phase_deg * 5.0).rem_euclid(360.0);
        let phase_delta = (target_firing_phase - self.firing_phase_deg).rem_euclid(360.0);
        let firing_cycle_samples = self.source_sample_rate as f64 * 12.0 / self.rpm_anchor as f64;
        self.cursor = phase_delta as f64 / 360.0 * firing_cycle_samples;
    }

    #[inline]
    fn render(&mut self, rpm: f32, output_sample_rate: u32) -> (f32, f32) {
        let tonal = sinc_sample(&self.tonal, self.cursor);
        let residual = sinc_sample(&self.residual, self.cursor);
        self.advance(rpm, output_sample_rate);
        (tonal, residual)
    }

    /// Playback ratio in source samples per output sample.
    #[inline]
    fn rate(&self, rpm: f32, output_sample_rate: u32) -> f64 {
        rpm as f64 / self.rpm_anchor as f64 * self.source_sample_rate as f64
            / output_sample_rate as f64
    }

    /// AUD-07: resample through a precomputed table, then advance with the
    /// caller-supplied ratio (computed once per zone per sample in `process`).
    #[inline]
    fn render_tabled(
        &mut self,
        rate: f64,
        table: &[f32],
        table_phases: usize,
        tap_lo: isize,
        taps: usize,
    ) -> (f32, f32) {
        let tonal = sinc_sample_tabled(&self.tonal, self.cursor, table, table_phases, tap_lo, taps);
        let residual =
            sinc_sample_tabled(&self.residual, self.cursor, table, table_phases, tap_lo, taps);
        self.advance_with_rate(rate);
        (tonal, residual)
    }

    /// AUD-06: advances the loop cursor without resampling, so a suspended
    /// zone keeps its mechanical phase and resumes sample-aligned. Uses the
    /// identical rate formula as `render`, keeping a single phase authority.
    #[inline]
    fn advance(&mut self, rpm: f32, output_sample_rate: u32) {
        self.advance_with_rate(self.rate(rpm, output_sample_rate));
    }

    /// Cursor advance for an already-computed ratio (AUD-07 callers).
    #[inline]
    fn advance_with_rate(&mut self, rate: f64) {
        self.cursor = (self.cursor + rate).rem_euclid(self.tonal.len() as f64);
    }
}

pub struct ThreeZoneSampleLayer {
    output_sample_rate: u32,
    config: ThreeZoneSampleLayerConfig,
    /// ON zones in anchor order. Legacy schema-1 banks hold exactly three
    /// single-member zones; schema-2 banks hold N zones with variant groups.
    zones: Vec<OnZone>,
    /// OFF zones in anchor order (empty when the bank carries no OFF layer).
    /// OFF members render dry; their `processor` is never used.
    off_zones: Vec<OnZone>,
    /// True for legacy schema-1 banks: exact original 3-zone curves and
    /// suspension geometry. False enables the generalized neighbor-blend
    /// curves for schema-2 banks.
    legacy_three_zone: bool,
    /// Diagnostic labels aligned with `SampleLayerFrame.zone_weights`:
    /// `"on:<key>"` for every ON source, then `"off:<key>"` for OFF sources.
    source_labels: Vec<String>,
    phase_aligned: bool,
    /// AUD-06: count of suspended (cursor-only) zone renders since construction.
    /// Control-thread diagnostic for measuring skipped work; written only on the
    /// audio thread, never allocated, locked, or logged in the callback.
    suspended_zone_samples: u64,
    /// AUD-07a: precomputed radius-4 kernel (transparent accelerator).
    tabled_narrow: Vec<f32>,
    off_smoothed: f32,
}

impl ThreeZoneSampleLayer {
    pub fn load_directory(
        output_sample_rate: u32,
        directory: &Path,
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        let manifest_path = directory.join("manifest.json");
        if manifest_path.is_file() {
            let raw = fs::read_to_string(&manifest_path).map_err(|error| {
                format!("cannot read {}: {error}", manifest_path.display())
            })?;
            // Peek at the schema version without committing to a shape.
            let peek: serde_json::Value = serde_json::from_str(&raw).map_err(|error| {
                format!("invalid {}: {error}", manifest_path.display())
            })?;
            let schema = peek
                .get("schema_version")
                .and_then(|value| value.as_u64())
                .unwrap_or(0);
            if schema == 2 {
                return Self::load_schema2(output_sample_rate, directory, config);
            }
            // Any other manifest.json (schema 1 GF509 banks carry one too)
            // falls through to the legacy suffix discovery below.
        }
        let entries = fs::read_dir(directory)
            .map_err(|error| format!("cannot read {}: {error}", directory.display()))?
            .map(|entry| {
                entry
                    .map(|entry| entry.path())
                    .map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        let off_path = entries.iter().find(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.ends_with(".off-layer.json"))
        }).cloned();
        let mut metadata_paths: Vec<PathBuf> = entries
            .into_iter()
            .filter(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.ends_with(".sample-layer.json"))
            })
            .collect();
        metadata_paths.sort();
        let metadata_paths: [PathBuf; ZONE_COUNT] =
            metadata_paths.try_into().map_err(|paths: Vec<_>| {
                format!(
                    "ThreeZoneSampleLayer expected 3 metadata files in {}, found {}",
                    directory.display(),
                    paths.len()
                )
            })?;
        let off_zone = off_path.as_deref().map(SampleZone::load).transpose()?;
        Self::load_with_off(output_sample_rate, metadata_paths, off_zone, config)
    }

    pub fn load(
        output_sample_rate: u32,
        metadata_paths: [PathBuf; ZONE_COUNT],
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        Self::load_with_off(output_sample_rate, metadata_paths, None, config)
    }

    fn load_with_off(
        output_sample_rate: u32,
        metadata_paths: [PathBuf; ZONE_COUNT],
        off_zone: Option<SampleZone>,
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        if !(8_000..=192_000).contains(&output_sample_rate) {
            return Err(format!(
                "sample-layer output rate out of range: {output_sample_rate}"
            ));
        }
        let config = config.validate()?;
        let mut loaded = metadata_paths
            .iter()
            .map(|path| SampleZone::load(path))
            .collect::<Result<Vec<_>, _>>()?;
        loaded.sort_by(|left, right| left.rpm_anchor.total_cmp(&right.rpm_anchor));
        if loaded
            .windows(2)
            .any(|pair| pair[1].rpm_anchor - pair[0].rpm_anchor < 100.0)
        {
            return Err("sample-layer RPM anchors must be distinct and ordered".into());
        }
        if config.max_fade_start_rpm < loaded[1].rpm_anchor
            || config.max_full_rpm > loaded[2].rpm_anchor
        {
            return Err("max-zone crossfade window must stay between med and max anchors".into());
        }
        let zones: Vec<OnZone> = loaded
            .into_iter()
            .enumerate()
            .map(|(index, zone)| {
                let key = format!("legacy-{index}");
                OnZone {
                    anchor: zone.rpm_anchor,
                    members: vec![ZoneMember {
                        key: key.clone(),
                        zone,
                        variant_group: None,
                        variant_position: 0,
                        variant_total: 1,
                    }],
                    processor: ZoneMidProcessor::new(
                        index,
                        output_sample_rate as f32,
                        !config.disable_sample_rasp,
                        config.zone_trim_db[index.min(ZONE_TRIM_SLOTS - 1)],
                    ),
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                }
            })
            .collect();
        let off_zones = off_zone
            .into_iter()
            .map(|zone| {
                let anchor = zone.rpm_anchor;
                OnZone {
                    anchor,
                    members: vec![ZoneMember {
                        key: "legacy-off".to_string(),
                        zone,
                        variant_group: None,
                        variant_position: 0,
                        variant_total: 1,
                    }],
                    processor: ZoneMidProcessor::new(1, output_sample_rate as f32, true, 0.0),
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                }
            })
            .collect();
        Self::finish(
            output_sample_rate,
            config,
            zones,
            off_zones,
            true,
            vec![
                "on:legacy-0".to_string(),
                "on:legacy-1".to_string(),
                "on:legacy-2".to_string(),
                "off:legacy-off".to_string(),
            ],
        )
    }

    /// Schema-2 experimental bank: roles and variant groups come from the
    /// manifest, never from filename suffixes.
    fn load_schema2(
        output_sample_rate: u32,
        directory: &Path,
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        if !(8_000..=192_000).contains(&output_sample_rate) {
            return Err(format!(
                "sample-layer output rate out of range: {output_sample_rate}"
            ));
        }
        let config = config.validate()?;
        let manifest_path = directory.join("manifest.json");
        let raw = fs::read_to_string(&manifest_path)
            .map_err(|error| format!("cannot read {}: {error}", manifest_path.display()))?;
        let manifest: ManifestSchema2 = serde_json::from_str(&raw)
            .map_err(|error| format!("invalid {}: {error}", manifest_path.display()))?;
        if manifest.schema_version != 2
            || !matches!(
                manifest.key.as_str(),
                "v10_f2002_experimental" | "v10_f2002_sampled"
            )
        {
            return Err("sample-layer schema-2 manifest key/version mismatch".into());
        }
        if (manifest.output_gain - 0.61).abs() > f32::EPSILON {
            return Err("schema-2 manifest output gain does not match 0.61".into());
        }
        if manifest.samples.is_empty() {
            return Err("schema-2 manifest carries no ON samples".into());
        }
        if manifest.off_samples.is_empty() {
            return Err("schema-2 manifest carries no OFF samples".into());
        }
        let mut on_members = Self::load_manifest_members(
            directory,
            &manifest.samples,
            "on",
            output_sample_rate,
        )?;
        let mut off_members = Self::load_manifest_members(
            directory,
            &manifest.off_samples,
            "off",
            output_sample_rate,
        )?;
        on_members.sort_by(|left: &ZoneMember, right: &ZoneMember| {
            left.zone.rpm_anchor.total_cmp(&right.zone.rpm_anchor)
        });
        off_members.sort_by(|left: &ZoneMember, right: &ZoneMember| {
            left.zone.rpm_anchor.total_cmp(&right.zone.rpm_anchor)
        });
        let mut zones = Self::group_members(
            on_members,
            output_sample_rate,
            true,
            &config,
        )?;
        let off_zones = Self::group_members(off_members, output_sample_rate, false, &config)?;
        // DSP assignment by function: low -> low processor, intermediates ->
        // med, top -> max. The max rasp boost must not leak onto every source.
        let last = zones.len() - 1;
        for (index, zone) in zones.iter_mut().enumerate() {
            let processor_kind = if index == 0 {
                0
            } else if index == last {
                2
            } else {
                1
            };
            zone.processor = ZoneMidProcessor::new(
                processor_kind,
                output_sample_rate as f32,
                !config.disable_sample_rasp,
                config.zone_trim_db[index.min(ZONE_TRIM_SLOTS - 1)],
            );
        }
        let mut labels = Vec::new();
        for zone in &zones {
            for member in &zone.members {
                labels.push(format!("on:{}", member.key));
            }
        }
        for zone in &off_zones {
            for member in &zone.members {
                labels.push(format!("off:{}", member.key));
            }
        }
        Self::finish(
            output_sample_rate,
            config,
            zones,
            off_zones,
            false,
            labels,
        )
    }

    fn load_manifest_members(
        directory: &Path,
        entries: &[ManifestSample],
        role: &str,
        output_sample_rate: u32,
    ) -> Result<Vec<ZoneMember>, String> {
        let mut members = Vec::with_capacity(entries.len());
        for entry in entries {
            if entry.role != role {
                return Err(format!(
                    "schema-2 entry {} declares role {}, expected {role}",
                    entry.key, entry.role
                ));
            }
            if !entry.native_rpm.is_finite()
                || !(1_000.0..=25_000.0).contains(&entry.native_rpm)
            {
                return Err(format!("schema-2 entry {} has an invalid anchor", entry.key));
            }
            if entry.sample_rate != output_sample_rate
                || entry.channels != 1
                || entry.loop_frames < 32
            {
                return Err(format!(
                    "schema-2 entry {} has incompatible format metadata",
                    entry.key
                ));
            }
            for name in [&entry.metadata, &entry.tonal, &entry.residual] {
                let candidate = Path::new(name);
                if candidate.is_absolute() || candidate.components().count() != 1 {
                    return Err(format!("schema-2 asset path must be local: {name}"));
                }
                if !directory.join(name).is_file() {
                    return Err(format!("schema-2 asset missing: {name}"));
                }
            }
            verify_file_sha256(&directory.join(&entry.tonal), &entry.tonal_sha256)?;
            verify_file_sha256(&directory.join(&entry.residual), &entry.residual_sha256)?;
            let zone = SampleZone::load(&directory.join(&entry.metadata))?;
            if (zone.rpm_anchor - entry.native_rpm).abs() > 1.0 {
                return Err(format!(
                    "schema-2 entry {} anchor mismatch: manifest {} vs prepared {}",
                    entry.key, entry.native_rpm, zone.rpm_anchor
                ));
            }
            let (variant_group, variant_position, variant_total) = match &entry.variant {
                None => (None, 0, 1),
                Some(variant) => {
                    if variant.count < 2
                        || variant.position >= variant.count
                        || variant.group.is_empty()
                    {
                        return Err(format!(
                            "schema-2 entry {} has an invalid variant record",
                            entry.key
                        ));
                    }
                    (
                        Some(variant.group.clone()),
                        variant.position,
                        variant.count,
                    )
                }
            };
            members.push(ZoneMember {
                key: entry.key.clone(),
                zone,
                variant_group,
                variant_position,
                variant_total,
            });
        }
        Ok(members)
    }

    /// Sorts members (already anchor-sorted) into zones. Variant-group members
    /// must form one contiguous run; every other member becomes a solo zone.
    /// Crossfade geometry uses the member-anchor mean; playback rates always
    /// use each member's own anchor.
    fn group_members(
        members: Vec<ZoneMember>,
        output_sample_rate: u32,
        is_on: bool,
        config: &ThreeZoneSampleLayerConfig,
    ) -> Result<Vec<OnZone>, String> {
        let _ = config;
        if members.is_empty() {
            return Err("schema-2 role collection is empty".into());
        }
        // Validate variant records before grouping.
        {
            let mut seen: std::collections::BTreeMap<String, Vec<usize>> =
                std::collections::BTreeMap::new();
            for (index, member) in members.iter().enumerate() {
                if let Some(group) = &member.variant_group {
                    seen.entry(group.clone()).or_default().push(index);
                } else if member.variant_total != 1 || member.variant_position != 0 {
                    return Err(format!("member {} has an inconsistent solo record", member.key));
                }
            }
            for (group, indices) in &seen {
                let Some(first) = members
                    .iter()
                    .find(|member| member.variant_group.as_deref() == Some(group.as_str()))
                else {
                    return Err(format!("variant group {group} has no members"));
                };
                if indices.len() != first.variant_total {
                    return Err(format!("variant group {group} is incomplete"));
                }
                let positions: std::collections::BTreeSet<usize> = indices
                    .iter()
                    .map(|&index| members[index].variant_position)
                    .collect();
                if positions.len() != indices.len()
                    || !positions.iter().copied().eq(0..indices.len())
                {
                    return Err(format!("variant group {group} positions are not 0..count"));
                }
                if indices.windows(2).any(|pair| pair[1] != pair[0] + 1) {
                    return Err(format!("variant group {group} members are not contiguous"));
                }
            }
        }
        let mut zones: Vec<OnZone> = Vec::new();
        // Rebuild by draining through a queue (avoids partial moves).
        let mut queue: std::collections::VecDeque<ZoneMember> = members.into();
        while let Some(first) = queue.pop_front() {
            if let Some(group) = first.variant_group.clone() {
                let mut run = vec![first];
                while queue
                    .front()
                    .is_some_and(|member| member.variant_group.as_deref() == Some(group.as_str()))
                {
                    match queue.pop_front() {
                        Some(member) => run.push(member),
                        None => break,
                    }
                }
                let anchor =
                    run.iter().map(|member| member.zone.rpm_anchor).sum::<f32>() / run.len() as f32;
                zones.push(OnZone {
                    anchor,
                    members: run,
                    processor: ZoneMidProcessor::new(1, output_sample_rate as f32, true, 0.0),
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                });
            } else {
                let anchor = first.zone.rpm_anchor;
                zones.push(OnZone {
                    anchor,
                    members: vec![first],
                    processor: ZoneMidProcessor::new(1, output_sample_rate as f32, true, 0.0),
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                });
            }
        }
        // Guard against degenerate geometry (equal anchors inside a group are
        // fine; equal zone anchors are not).
        let anchors: Vec<f32> = zones.iter().map(|zone| zone.anchor).collect();
        if anchors.windows(2).any(|pair| pair[1] - pair[0] < 1.0e-3) {
            return Err("schema-2 zone anchors must be distinct and ordered".into());
        }
        // Variant-interpolation spans: the full group span between neighbor
        // midpoints, extended by half a gap at the collection edges.
        let bounds = zone_boundaries(&anchors);
        for (zone_index, zone) in zones.iter_mut().enumerate() {
            let (lo, hi) = bounds[zone_index];
            zone.variant_span = (lo, hi);
        }
        // Silence unused role flag (DSP kind is assigned by the caller for ON;
        // OFF zones share the OnZone shape without a processor use).
        let _ = is_on;
        Ok(zones)
    }

    fn finish(
        output_sample_rate: u32,
        config: ThreeZoneSampleLayerConfig,
        zones: Vec<OnZone>,
        off_zones: Vec<OnZone>,
        legacy_three_zone: bool,
        source_labels: Vec<String>,
    ) -> Result<Self, String> {
        Ok(Self {
            output_sample_rate,
            config,
            zones,
            off_zones,
            legacy_three_zone,
            source_labels,
            phase_aligned: false,
            suspended_zone_samples: 0,
            tabled_narrow: build_sinc_table(SINC_TABLE_PHASES, SINC_RADIUS),
            off_smoothed: 0.0,
        })
    }

    pub fn reset_phase(&mut self, crank_phase_deg: f32) -> Result<(), String> {
        if !crank_phase_deg.is_finite() {
            return Err("sample-layer crank phase must be finite".into());
        }
        for zone in &mut self.zones {
            for member in &mut zone.members {
                member.zone.align(crank_phase_deg);
            }
        }
        for zone in &mut self.off_zones {
            for member in &mut zone.members {
                member.zone.align(crank_phase_deg);
            }
        }
        self.off_smoothed = 0.0;
        self.phase_aligned = true;
        Ok(())
    }

    pub fn rpm_anchors(&self) -> Vec<f32> {
        self.zones.iter().map(|zone| zone.anchor).collect()
    }

    /// Diagnostic labels aligned with `SampleLayerFrame.zone_weights`.
    pub fn source_labels(&self) -> &[String] {
        &self.source_labels
    }

    /// Number of OFF sources (members across all OFF zones).
    pub fn off_source_count(&self) -> usize {
        self.off_zones
            .iter()
            .map(|zone| zone.members.len())
            .sum()
    }

    /// AUD-06 diagnostic: zone renders skipped (cursor advanced, no DSP) since
    /// construction. Read between blocks, never in the callback.
    pub fn suspended_zone_samples(&self) -> u64 {
        self.suspended_zone_samples
    }

    /// Resamples one member through the configured kernel path and advances
    /// its cursor. Split out so ON and OFF collections share one authority.
    #[inline]
    fn render_member(
        member: &mut ZoneMember,
        rpm: f32,
        output_sample_rate: u32,
        tabled: bool,
        tabled_narrow: &[f32],
    ) -> (f32, f32) {
        let zone = &mut member.zone;
        if !tabled {
            return zone.render(rpm, output_sample_rate);
        }
        let rate = zone.rate(rpm, output_sample_rate);
        zone.render_tabled(
            rate,
            tabled_narrow,
            SINC_TABLE_PHASES,
            -SINC_RADIUS + 1,
            (2 * SINC_RADIUS) as usize,
        )
    }

    #[inline]
    pub fn process(&mut self, input: SampleLayerInput) -> Result<SampleLayerFrame, String> {
        let mut frame = SampleLayerFrame::default();
        self.process_into(input, &mut frame)?;
        Ok(frame)
    }

    /// Allocation-free variant of [`Self::process`]: fills a caller-owned frame so
    /// the audio callback can reuse one buffer instead of allocating per sample.
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
        if self.legacy_three_zone {
            return self.process_legacy_into(input, out);
        }
        *out = self.process_multi(input)?;
        Ok(())
    }

    /// Exact legacy behavior for schema-1 GF509 banks: original 3-zone
    /// curves, suspension geometry and single OFF stem.
    fn process_legacy_into(
        &mut self,
        input: SampleLayerInput,
        out: &mut SampleLayerFrame,
    ) -> Result<(), String> {
        if self.zones.len() != ZONE_COUNT {
            return Err("legacy bank requires exactly three ON zones".to_string());
        }
        let mut legacy = [0.0f32; ZONE_COUNT];
        for (slot, zone) in legacy.iter_mut().zip(self.zones.iter()) {
            *slot = zone.anchor;
        }
        let weights = zone_weights(
            input.rpm,
            legacy,
            self.config.max_fade_start_rpm,
            self.config.max_full_rpm,
        );
        let mut tonal = 0.0;
        let mut residual = 0.0;
        let mut tonal_rasp = 0.0;
        let mut residual_rasp = 0.0;
        let output_sample_rate = self.output_sample_rate;
        let suspend = self.config.suspend_inaudible_zones;
        let margin = self.config.suspend_margin_rpm;
        let tabled = self.config.use_tabled_sinc;
        let fade_start = self.config.max_fade_start_rpm;
        let fade_full = self.config.max_full_rpm;
        let label_count = self.source_labels.len();
        out.zone_weights.clear();
        out.zone_weights.resize(label_count, 0.0);
        for (index, zone) in self.zones.iter_mut().enumerate() {
            let weight = weights[index];
            // AUD-06 (fix.txt): suspend only in deep silence — zero weight AND
            // the RPM a full margin outside every crossfade this zone takes
            // part in. Zones approaching a fade keep rendering so their filters
            // track and are converged at fade entry; the fade itself then
            // suppresses any residual. Cursor always advances (phase).
            let deep_silence = weight == 0.0
                && match index {
                    0 => input.rpm > legacy[1] + margin,
                    1 => {
                        input.rpm < legacy[0] - margin
                            || (input.rpm > legacy[1] + margin
                                && input.rpm < fade_start - margin)
                            || input.rpm > fade_full + margin
                    }
                    _ => input.rpm < fade_start - margin,
                };
            let member = &mut zone.members[0];
            if suspend && deep_silence {
                member.zone.advance(input.rpm, output_sample_rate);
                self.suspended_zone_samples += 1;
                continue;
            }
            let (zone_tonal, zone_residual) =
                Self::render_member(member, input.rpm, output_sample_rate, tabled, &self.tabled_narrow);
            let (zone_tonal, zone_residual, zone_tonal_rasp, zone_residual_rasp) =
                zone.processor.process(
                    zone_tonal,
                    zone_residual,
                    input.rpm,
                    self.output_sample_rate as f32,
                );
            tonal += zone_tonal * weight;
            residual += zone_residual * weight;
            tonal_rasp += zone_tonal_rasp * weight;
            residual_rasp += zone_residual_rasp * weight;
            out.zone_weights[index] = weight;
        }
        let charge = input.load * (0.35 + 0.65 * input.throttle);
        let tonal_gain = self.config.tonal_gain_closed
            + (self.config.tonal_gain_loaded - self.config.tonal_gain_closed) * charge;
        let residual_gain = (self.config.residual_gain_closed
            + (self.config.residual_gain_loaded - self.config.residual_gain_closed) * charge)
            * self.config.residual_gain_scale;
        tonal *= tonal_gain;
        residual *= residual_gain;
        let max_rasp = tonal_rasp * tonal_gain + residual_rasp * residual_gain;

        let mut off_throttle_stem = 0.0;
        if let Some(off_zone) = self.off_zones.first_mut() {
            let off_weight = off_throttle_weight(input, self.config.off_throttle_gain);
            // fix.txt: slew the weight itself (32 samples) so a 0 -> audible
            // jump on a throttle slam cannot click against filter state.
            // Steady weights pass through untouched after the slew.
            let target = off_weight;
            let delta = (target - self.off_smoothed).clamp(-1.0 / 32.0, 1.0 / 32.0);
            self.off_smoothed += delta;
            let off_weight = self.off_smoothed;
            // AUD-06: same suspension for the off stem (stateless beyond its
            // cursor, so reactivation is exact).
            let member = &mut off_zone.members[0];
            let (off_tonal, off_residual) = if suspend && off_weight == 0.0 {
                member.zone.advance(input.rpm, output_sample_rate);
                self.suspended_zone_samples += 1;
                (0.0, 0.0)
            } else {
                Self::render_member(
                    member,
                    input.rpm,
                    output_sample_rate,
                    tabled,
                    &self.tabled_narrow,
                )
            };
            tonal += off_tonal * off_weight;
            residual += off_residual * off_weight;
            off_throttle_stem = (off_tonal + off_residual) * off_weight;
            let off_index = self.zones.len();
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

    /// Generalized schema-2 behavior: N ON zones with variant groups and M
    /// OFF zones. Same gains, suspension policy and headroom handling as the
    /// legacy path; only the bank geometry and its distribution change.
    fn process_multi(&mut self, input: SampleLayerInput) -> Result<SampleLayerFrame, String> {
        let anchors = self.rpm_anchors();
        let zone_weights = multi_zone_weights(input.rpm, &anchors);
        let mut tonal = 0.0;
        let mut residual = 0.0;
        let mut tonal_rasp = 0.0;
        let mut residual_rasp = 0.0;
        let output_sample_rate = self.output_sample_rate;
        let output_sample_rate_f32 = self.output_sample_rate as f32;
        let suspend = self.config.suspend_inaudible_zones;
        let margin = self.config.suspend_margin_rpm;
        let tabled = self.config.use_tabled_sinc;
        let bounds = zone_boundaries(&anchors);
        let mut frame_weights = vec![0.0f32; self.source_labels.len()];
        let mut weight_cursor = 0usize;
        for (index, zone) in self.zones.iter_mut().enumerate() {
            let weight = zone_weights[index];
            let (span_lo, span_hi) = bounds[index];
            // Generalized deep silence: zero weight AND the RPM a full margin
            // outside every crossfade this zone takes part in.
            let deep_silence = weight == 0.0
                && (input.rpm < span_lo - margin || input.rpm > span_hi + margin);
            // Variant interpolation over the group span: deterministic,
            // power-normalized, continuous cursors per member.
            let variant_weights = variant_weights(input.rpm, zone);
            if suspend && deep_silence {
                for member in &mut zone.members {
                    member.zone.advance(input.rpm, output_sample_rate);
                    self.suspended_zone_samples += 1;
                }
                weight_cursor += zone.members.len();
                continue;
            }
            let mut mixed_tonal = 0.0;
            let mut mixed_residual = 0.0;
            for (member, variant_weight) in
                zone.members.iter_mut().zip(variant_weights.iter())
            {
                // Exact skip on the variant factor only: a zero-variant member
                // contributes nothing to the zone mix (so shared DSP state is
                // unaffected) and its only state is the cursor, which still
                // advances (phase). The zone factor must NOT gate the skip: a
                // zero-weight zone inside the margin still renders for filter
                // tracking, and that needs the full member mix.
                if *variant_weight == 0.0 {
                    member.zone.advance(input.rpm, output_sample_rate);
                    self.suspended_zone_samples += 1;
                    frame_weights[weight_cursor] = 0.0;
                    weight_cursor += 1;
                    continue;
                }
                let member_weight = weight * variant_weight;
                let (member_tonal, member_residual) = Self::render_member(
                    member,
                    input.rpm,
                    output_sample_rate,
                    tabled,
                    &self.tabled_narrow,
                );
                mixed_tonal += member_tonal * variant_weight;
                mixed_residual += member_residual * variant_weight;
                frame_weights[weight_cursor] = member_weight;
                weight_cursor += 1;
            }
            let (zone_tonal, zone_residual, zone_tonal_rasp, zone_residual_rasp) =
                zone.processor.process(
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
        let tonal_gain = self.config.tonal_gain_closed
            + (self.config.tonal_gain_loaded - self.config.tonal_gain_closed) * charge;
        let residual_gain = (self.config.residual_gain_closed
            + (self.config.residual_gain_loaded - self.config.residual_gain_closed) * charge)
            * self.config.residual_gain_scale;
        tonal *= tonal_gain;
        residual *= residual_gain;
        let max_rasp = tonal_rasp * tonal_gain + residual_rasp * residual_gain;

        // Independent OFF selection over its own anchors, with the existing
        // retention weight, throttle smoothing and no lift discontinuity.
        let mut off_throttle_stem = 0.0;
        if !self.off_zones.is_empty() {
            let off_anchors: Vec<f32> =
                self.off_zones.iter().map(|zone| zone.anchor).collect();
            let off_zone_weights = multi_zone_weights(input.rpm, &off_anchors);
            let target = off_throttle_weight(input, self.config.off_throttle_gain);
            let delta = (target - self.off_smoothed).clamp(-1.0 / 32.0, 1.0 / 32.0);
            self.off_smoothed += delta;
            let off_weight = self.off_smoothed;
            let mut mixed_tonal = 0.0;
            let mut mixed_residual = 0.0;
            for (zone, zone_weight) in
                self.off_zones.iter_mut().zip(off_zone_weights.iter())
            {
                let variant_weights = variant_weights(input.rpm, zone);
                if suspend && off_weight == 0.0 {
                    for member in &mut zone.members {
                        member.zone.advance(input.rpm, output_sample_rate);
                        self.suspended_zone_samples += 1;
                    }
                    weight_cursor += zone.members.len();
                    continue;
                }
                for (member, variant_weight) in
                    zone.members.iter_mut().zip(variant_weights.iter())
                {
                    let member_weight = off_weight * zone_weight * variant_weight;
                    if member_weight == 0.0 {
                        // Exact skip: OFF members render dry with no shared
                        // state, so a zero-weight member is pure waste; the
                        // cursor still advances (phase).
                        member.zone.advance(input.rpm, output_sample_rate);
                        self.suspended_zone_samples += 1;
                        frame_weights[weight_cursor] = 0.0;
                        weight_cursor += 1;
                        continue;
                    }
                    let (member_tonal, member_residual) = Self::render_member(
                        member,
                        input.rpm,
                        output_sample_rate,
                        tabled,
                        &self.tabled_narrow,
                    );
                    mixed_tonal += member_tonal * zone_weight * variant_weight;
                    mixed_residual += member_residual * zone_weight * variant_weight;
                    frame_weights[weight_cursor] = member_weight;
                    weight_cursor += 1;
                }
            }
            tonal += mixed_tonal * off_weight;
            residual += mixed_residual * off_weight;
            off_throttle_stem = (mixed_tonal + mixed_residual) * off_weight;
        }

        Ok(SampleLayerFrame {
            tonal,
            residual,
            max_rasp,
            off_throttle: off_throttle_stem,
            output: tonal + residual,
            zone_weights: frame_weights,
        })
    }
}

#[inline]
fn off_throttle_weight(input: SampleLayerInput, gain: f32) -> f32 {    let overrun = ((0.30 - input.throttle) / 0.30).clamp(0.0, 1.0);
    let low_load = ((0.40 - input.load) / 0.40).clamp(0.0, 1.0);
    // A high absolute clutch load can still be engine retention. Use signed
    // torque to preserve that off character. Both the low-load coast term and
    // the retention term require an engaged clutch: a coherent free-rev packet
    // from physics carries clutch=0/load=0, so it must stay silent in this
    // layer instead of falling back to low_load=1. TC is already represented
    // by the physical load packet and is deliberately not reapplied.
    let retention = (-input.normalized_engine_torque).max(0.0) * input.clutch_engagement;
    let low_load_engaged = low_load * input.clutch_engagement;
    overrun * low_load_engaged.max(retention) * gain
}

fn verify_file_sha256(path: &Path, expected: &str) -> Result<(), String> {
    use sha2::{Digest, Sha256};
    let bytes = fs::read(path)
        .map_err(|error| format!("cannot read sample asset {}: {error}", path.display()))?;
    let actual = format!("{:x}", Sha256::digest(&bytes));
    if actual != *expected {
        return Err(format!("sample asset hash mismatch: {}", path.display()));
    }
    Ok(())
}

/// Crossfade boundaries for N sorted zone anchors: zone `i` blends with its
/// neighbors over `[bounds[i].0, bounds[i].1]`, where interior edges are
/// neighbor midpoints and collection edges extend half a gap outward.
fn zone_boundaries(anchors: &[f32]) -> Vec<(f32, f32)> {
    let count = anchors.len();
    let mut bounds = Vec::with_capacity(count);
    for index in 0..count {
        let lo = if index == 0 {
            if count > 1 {
                anchors[0] - (anchors[1] - anchors[0]) * 0.5
            } else {
                f32::NEG_INFINITY
            }
        } else {
            (anchors[index - 1] + anchors[index]) * 0.5
        };
        let hi = if index + 1 == count {
            if count > 1 {
                anchors[count - 1] + (anchors[count - 1] - anchors[count - 2]) * 0.5
            } else {
                f32::INFINITY
            }
        } else {
            (anchors[index] + anchors[index + 1]) * 0.5
        };
        bounds.push((lo, hi));
    }
    bounds
}

/// Generalized neighbor equal-power crossfade over N sorted zone anchors.
/// Below the first or above the last anchor the extreme zone holds.
fn multi_zone_weights(rpm: f32, anchors: &[f32]) -> Vec<f32> {
    let count = anchors.len();
    let mut weights = vec![0.0f32; count];
    if count == 0 {
        return weights;
    }
    if count == 1 || rpm <= anchors[0] {
        weights[0] = 1.0;
        return weights;
    }
    if rpm >= anchors[count - 1] {
        weights[count - 1] = 1.0;
        return weights;
    }
    for index in 0..count - 1 {
        if rpm >= anchors[index] && rpm < anchors[index + 1] {
            let span = (anchors[index + 1] - anchors[index]).max(1.0e-3);
            let t = ((rpm - anchors[index]) / span).clamp(0.0, 1.0);
            let theta = t * std::f32::consts::FRAC_PI_2;
            weights[index] = theta.cos();
            weights[index + 1] = theta.sin();
            return weights;
        }
    }
    weights[count - 1] = 1.0;
    weights
}

/// Variant interpolation inside one zone over its span: deterministic,
/// power-normalized equal-power chain. Solo members return weight 1.
/// Members keep continuous cursors and their own anchor rates.
fn variant_weights(rpm: f32, zone: &OnZone) -> Vec<f32> {
    let count = zone.members.len();
    if count == 1 {
        return vec![1.0];
    }
    let (lo, hi) = zone.variant_span;
    let span = (hi - lo).max(1.0e-3);
    let t = ((rpm - lo) / span).clamp(0.0, 1.0) * (count - 1) as f32;
    let segment = (t.floor() as usize).min(count - 2);
    let frac = t - segment as f32;
    let theta = frac * std::f32::consts::FRAC_PI_2;
    let mut weights = vec![0.0f32; count];
    weights[segment] = theta.cos();
    weights[segment + 1] = theta.sin();
    weights
}

#[inline]
fn zone_weights(    rpm: f32,
    anchors: [f32; ZONE_COUNT],
    max_fade_start_rpm: f32,
    max_full_rpm: f32,
) -> [f32; ZONE_COUNT] {
    if rpm <= anchors[0] {
        return [1.0, 0.0, 0.0];
    }
    if rpm < anchors[1] {
        let t = ((rpm - anchors[0]) / (anchors[1] - anchors[0])).clamp(0.0, 1.0);
        let theta = t * std::f32::consts::FRAC_PI_2;
        return [theta.cos(), theta.sin(), 0.0];
    }
    if rpm < max_fade_start_rpm {
        return [0.0, 1.0, 0.0];
    }
    if rpm < max_full_rpm {
        let t = ((rpm - max_fade_start_rpm) / (max_full_rpm - max_fade_start_rpm)).clamp(0.0, 1.0);
        let theta = t * std::f32::consts::FRAC_PI_2;
        return [0.0, theta.cos(), theta.sin()];
    }
    [0.0, 0.0, 1.0]
}

#[inline]
fn sinc_sample(samples: &[f32], cursor: f64) -> f32 {
    let base = cursor.floor() as isize;
    let fraction = cursor - cursor.floor();
    let mut output = 0.0f64;
    let mut weight_sum = 0.0f64;
    for tap in -SINC_RADIUS + 1..=SINC_RADIUS {
        let distance = tap as f64 - fraction;
        let sinc = if distance.abs() < 1.0e-12 {
            1.0
        } else {
            (PI * distance).sin() / (PI * distance)
        };
        let normalized = distance / SINC_RADIUS as f64;
        let window = if normalized.abs() <= 1.0 {
            0.5 + 0.5 * (PI * normalized).cos()
        } else {
            0.0
        };
        let index = (base + tap).rem_euclid(samples.len() as isize) as usize;
        let weight = sinc * window;
        output += samples[index] as f64 * weight;
        weight_sum += weight;
    }
    (output / weight_sum.max(1.0e-12)) as f32
}

/// AUD-07a: windowed-sinc weight for one tap, factored so tables and the
/// reference evaluator share the exact kernel definition.
fn windowed_sinc_weight(distance: f64, radius: isize) -> f64 {
    let sinc = if distance.abs() < 1.0e-12 {
        1.0
    } else {
        (PI * distance).sin() / (PI * distance)
    };
    let normalized = distance / radius as f64;
    let window = if normalized.abs() <= 1.0 {
        0.5 + 0.5 * (PI * normalized).cos()
    } else {
        0.0
    };
    sinc * window
}

/// AUD-07a: precomputes one normalized kernel table row per fractional phase.
/// Entry layout is `[phase * taps + tap]`; each phase row sums to one, baking
/// in the reference's per-call `weight_sum` division. An extra closing row
/// holds the phase-1024 weights (identical to phase 0 one sample later, i.e.
/// taps shifted by one with a zero entering): interpolating toward it at the
/// wrap seam keeps tap alignment exact instead of blending toward the wrong
/// row. Built once on the control thread at layer construction; the callback
/// only reads the table.
fn build_sinc_table(phases: usize, radius: isize) -> Vec<f32> {
    let taps = (2 * radius) as usize;
    let tap_lo = -radius + 1;
    let mut table = vec![0.0f32; (phases + 1) * taps];
    for phase in 0..=phases {
        let fraction = phase as f64 / phases as f64;
        let mut sum = 0.0f64;
        for tap in 0..taps {
            // Beyond the seam the kernel is evaluated one sample later:
            // offset k at fraction 1 equals offset k-1 at fraction 0, and the
            // tap sliding out of the window is exactly zero.
            let weight = if phase < phases {
                let distance = (tap_lo + tap as isize) as f64 - fraction;
                windowed_sinc_weight(distance, radius)
            } else if tap > 0 {
                let distance = (tap_lo + tap as isize - 1) as f64;
                windowed_sinc_weight(distance, radius)
            } else {
                0.0
            };
            table[phase * taps + tap] = weight as f32;
            sum += weight;
        }
        let scale = 1.0 / sum.max(1.0e-12);
        for tap in 0..taps {
            table[phase * taps + tap] = (table[phase * taps + tap] as f64 * scale) as f32;
        }
    }
    table
}

/// AUD-07a: table lookup with linear phase interpolation. No sin/cos/division
/// in the loop — 8 lerps + multiply-accumulates per call. The table
/// carries one extra closing row so the wrap seam needs no branch: phase0
/// always has a valid successor with correct tap alignment.
#[inline]
fn sinc_sample_tabled(
    samples: &[f32],
    cursor: f64,
    table: &[f32],
    phases: usize,
    tap_lo: isize,
    taps: usize,
) -> f32 {
    let base = cursor.floor() as isize;
    let position = (cursor - cursor.floor()) * phases as f64;
    let phase0 = position.floor() as usize;
    debug_assert!(phase0 < phases);
    let phase0 = phase0.min(phases - 1);
    let phase1 = phase0 + 1;
    let blend = (position - position.floor()) as f32;
    let row0 = phase0 * taps;
    let row1 = phase1 * taps;
    let mut output = 0.0f32;
    for tap in 0..taps {
        let weight =
            table[row0 + tap] + (table[row1 + tap] - table[row0 + tap]) * blend;
        let index = (base + tap_lo + tap as isize).rem_euclid(samples.len() as isize) as usize;
        output += samples[index] * weight;
    }
    output
}

pub(crate) struct Pcm16Wav {
    pub(crate) sample_rate: u32,
    pub(crate) samples: Vec<f32>,
}

pub(crate) fn read_mono_pcm16(path: &Path) -> Result<Pcm16Wav, String> {
    let bytes =
        fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return Err(format!("not a RIFF/WAVE file: {}", path.display()));
    }
    let mut cursor = 12usize;
    let mut format = None;
    let mut data = None;
    while cursor + 8 <= bytes.len() {
        let id = &bytes[cursor..cursor + 4];
        let size = u32::from_le_bytes([bytes[cursor + 4], bytes[cursor + 5], bytes[cursor + 6], bytes[cursor + 7]]) as usize;
        let start = cursor + 8;
        let end = start
            .checked_add(size)
            .filter(|&end| end <= bytes.len())
            .ok_or_else(|| format!("truncated WAV chunk in {}", path.display()))?;
        if id == b"fmt " && size >= 16 {
            format = Some((
                u16::from_le_bytes([bytes[start], bytes[start + 1]]),
                u16::from_le_bytes([bytes[start + 2], bytes[start + 3]]),
                u32::from_le_bytes([bytes[start + 4], bytes[start + 5], bytes[start + 6], bytes[start + 7]]),
                u16::from_le_bytes([bytes[start + 14], bytes[start + 15]]),
            ));
        } else if id == b"data" {
            data = Some(&bytes[start..end]);
        }
        cursor = end + (size & 1);
    }
    let (encoding, channels, sample_rate, bits) =
        format.ok_or_else(|| format!("missing WAV fmt chunk: {}", path.display()))?;
    if encoding != 1 || channels != 1 || bits != 16 {
        return Err(format!("expected mono PCM16 WAV: {}", path.display()));
    }
    let data = data.ok_or_else(|| format!("missing WAV data chunk: {}", path.display()))?;
    if data.len() % 2 != 0 {
        return Err(format!("odd PCM16 data length: {}", path.display()));
    }
    let samples = data
        .chunks_exact(2)
        .map(|pair| i16::from_le_bytes([pair[0], pair[1]]) as f32 / 32_768.0)
        .collect();
    Ok(Pcm16Wav {
        sample_rate,
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zone_tonal_trims_validate_and_default_to_baseline() {
        let config = ThreeZoneSampleLayerConfig::default();
        assert_eq!(config.zone_trim_db, [0.0; ZONE_TRIM_SLOTS]);
        assert!(config.validate().is_ok());

        let mut invalid = config;
        invalid.zone_trim_db[3] = 13.0;
        assert!(invalid.validate().is_err());

        let mut valid = config;
        valid.zone_trim_db[0] = 2.0;
        valid.zone_trim_db[5] = -6.0;
        assert!(valid.validate().is_ok());
    }

    #[test]
    fn crossfades_have_constant_power() {
        let anchors = [7_500.0, 8_200.0, 15_000.0];
        for rpm in [7_500.0, 8_750.0, 10_000.0, 12_500.0, 15_000.0] {
            let weights = zone_weights(rpm, anchors, 10_000.0, 13_750.0);
            let power: f32 = weights.iter().map(|weight| weight * weight).sum();
            assert!((power - 1.0).abs() < 1.0e-6);
        }
        assert_eq!(zone_weights(9_999.0, anchors, 10_000.0, 13_750.0)[2], 0.0);
        assert_eq!(
            zone_weights(13_750.0, anchors, 10_000.0, 13_750.0),
            [0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn sinc_resampler_wraps_without_index_failure() {
        let source = [0.0, 1.0, 0.0, -1.0];
        for cursor in [0.0, 0.5, 3.75, 4.0, 9.25] {
            assert!(sinc_sample(&source, cursor).is_finite());
        }
        assert!(sinc_sample(&source, 0.0).abs() < 1.0e-6);
        assert!((sinc_sample(&source, 1.0) - 1.0).abs() < 1.0e-6);
    }

    fn deterministic_source(len: usize) -> Vec<f32> {
        // Pseudo-audio with harmonics + edge detail (no randomness allowed).
        (0..len)
            .map(|n| {
                let t = n as f64;
                (0.5 * (t * 0.11).sin() + 0.3 * (t * 0.031 + 1.0).sin() + 0.2 * (t * 0.53 + 2.0).sin())
                    as f32
            })
            .collect()
    }

    #[test]
    fn tabled_narrow_kernel_matches_reference_across_pitch_ratios() {
        // AUD-07a: pitch low, near one, and high, sweeping RPM both directions.
        // The table must stay transparent w.r.t. the evaluated kernel.
        let source = deterministic_source(4096);
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
        assert!(
            max_diff < 1.0e-4,
            "tabled radius-4 deviates {max_diff} from the evaluated kernel"
        );
    }

    #[test]
    fn off_weight_preserves_retention_at_fixed_rpm_without_double_tc() {
        let base = SampleLayerInput {
            rpm: 8_000.0,
            throttle: 0.0,
            load: 0.8,
            normalized_engine_torque: -0.8,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        let retention = off_throttle_weight(base, 0.20);
        assert!((retention - 0.16).abs() < 1.0e-6);

        // Coherent free-rev mapping from physics: clutch open implies load=0.
        // Must stay silent instead of falling back to low_load=1 (R1).
        let free_rev = SampleLayerInput {
            rpm: 8_000.0,
            throttle: 0.0,
            load: 0.0,
            normalized_engine_torque: 0.0,
            clutch_engagement: 0.0,
            crank_phase_deg: 0.0,
        };
        assert_eq!(off_throttle_weight(free_rev, 0.20), 0.0);

        // The legacy incoherent combination (clutch=0 with high load) also
        // stays silent because both terms are clutch-gated.
        let mut incoherent = base;
        incoherent.clutch_engagement = 0.0;
        assert_eq!(off_throttle_weight(incoherent, 0.20), 0.0);

        // TC is already reflected by the load supplied by physics. Changing
        // only an external TC control must not attenuate this off weight again.
        let mut same_state = base;
        let no_tc = off_throttle_weight(same_state, 0.20);
        same_state.load = 0.48;
        let tc_cut_load = off_throttle_weight(same_state, 0.20);
        assert_eq!(no_tc, tc_cut_load);
    }

    #[test]
    fn off_weight_silences_coherent_free_rev_but_keeps_low_load_coast() {
        // Free-rev: closed throttle with clutch open, no delivered load.
        // Overrun alone must not open the stem without clutch engagement.
        let free_rev = SampleLayerInput {
            rpm: 9_000.0,
            throttle: 0.0,
            load: 0.0,
            normalized_engine_torque: 0.0,
            clutch_engagement: 0.0,
            crank_phase_deg: 0.0,
        };
        assert_eq!(off_throttle_weight(free_rev, 0.20), 0.0);

        // Same throttle/load with clutch engaged keeps low-load coast audible.
        let coast = SampleLayerInput {
            clutch_engagement: 1.0,
            ..free_rev
        };
        assert!(off_throttle_weight(coast, 0.20) > 0.0);

        // Partial clutch scales the low-load term proportionally.
        let half = SampleLayerInput {
            clutch_engagement: 0.5,
            ..coast
        };
        let full = off_throttle_weight(coast, 0.20);
        let partial = off_throttle_weight(half, 0.20);
        assert!((partial - full * 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn off_weight_uses_low_load_when_torque_is_positive() {
        let input = SampleLayerInput {
            rpm: 8_000.0,
            throttle: 0.0,
            load: 0.2,
            normalized_engine_torque: 0.5,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        assert!((off_throttle_weight(input, 0.20) - 0.10).abs() < 1.0e-6);
    }

    #[test]
    fn tabled_kernel_cost_is_a_fraction_of_evaluated_sinc() {
        // AUD-07a CPU evidence at kernel level: time N resamples through the
        // evaluated kernel vs the tabled one (same trace of cursors/rates).
        // Full-route confirmation comes from aud_path_bench runs.
        let source = deterministic_source(8192);
        let narrow = build_sinc_table(SINC_TABLE_PHASES, SINC_RADIUS);
        let narrow_taps = (2 * SINC_RADIUS) as usize;
        let cursors: Vec<f64> = (0..20_000)
            .map(|n| (n as f64 * 1.37).rem_euclid(source.len() as f64))
            .collect();
        let time_it = |work: &mut dyn FnMut()| {
            let mut best = u128::MAX;
            for _ in 0..5 {
                let start = std::time::Instant::now();
                work();
                best = best.min(start.elapsed().as_nanos());
            }
            best
        };
        let mut sink = 0.0f32;
        let reference_ns = time_it(&mut || {
            for cursor in &cursors {
                sink += sinc_sample(&source, *cursor);
            }
        });
        let mut sink_tabled = 0.0f32;
        let tabled_ns = time_it(&mut || {
            for cursor in &cursors {
                sink_tabled += sinc_sample_tabled(
                    &source,
                    *cursor,
                    &narrow,
                    SINC_TABLE_PHASES,
                    -SINC_RADIUS + 1,
                    narrow_taps,
                );
            }
        });
        assert!(sink.is_finite() && sink_tabled.is_finite());
        assert!(sink.abs() > 1.0, "sinks must observe real work");
        println!(
            "AUD-07 kernel ns best-of-5 for 20k resamples: reference={reference_ns} tabled={tabled_ns}"
        );
        // Directional lock: the table must cost strictly less than evaluating
        // the kernel. Measured debug ratios live in the println above; the
        // shippable ratio comes from release aud_path_bench runs.
        assert!(
            tabled_ns < reference_ns,
            "tabled {tabled_ns}ns must cost less than evaluated {reference_ns}ns"
        );
    }

    #[test]
    fn tabled_default_stays_transparent_on_program_material() {
        // AUD-07a shipped-default transparency on the real bank: tabled vs
        // evaluated kernel across a full-range sweep (up/down + retention).
        // Transparency bound 1e-5 (-100 dBFS; measured 7.4e-7 on this sweep).
        use std::path::Path;
        let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_gf509");
        let base_config = ThreeZoneSampleLayerConfig::default();
        assert!(base_config.use_tabled_sinc, "shipped default must be tabled");
        let mut reference = ThreeZoneSampleLayer::load_directory(
            44_100,
            &assets,
            ThreeZoneSampleLayerConfig {
                use_tabled_sinc: false,
                ..base_config
            },
        )
        .unwrap();
        let mut tabled = ThreeZoneSampleLayer::load_directory(44_100, &assets, base_config).unwrap();
        let anchors = reference.rpm_anchors();
        let rpm_lo = (anchors[0] - 2_000.0).max(1_000.0);
        let rpm_hi = anchors[2] + 1_000.0;
        let mut max_diff = 0.0f32;
        let mut max_step_reference = 0.0f32;
        let mut max_step_tabled = 0.0f32;
        let mut previous_reference = 0.0f32;
        let mut previous_tabled = 0.0f32;
        for pass in [0, 1] {
            let steps = 20_000usize;
            for i in 0..steps {
                let u = i as f32 / (steps - 1) as f32;
                let rpm = if pass == 0 {
                    rpm_lo + (rpm_hi - rpm_lo) * u
                } else {
                    rpm_hi + (rpm_lo - rpm_hi) * u
                };
                let power = pass == 0;
                let input = SampleLayerInput {
                    rpm,
                    throttle: if power { 0.9 } else { 0.0 },
                    load: if power { 0.85 } else { 0.1 },
                    normalized_engine_torque: if power { 0.8 } else { -0.45 },
                    clutch_engagement: 1.0,
                    crank_phase_deg: 0.0,
                };
                let out_reference = reference.process(input).unwrap().output;
                let out_tabled = tabled.process(input).unwrap().output;
                assert!(out_reference.is_finite() && out_tabled.is_finite());
                max_diff = max_diff.max((out_reference - out_tabled).abs());
                if i > 0 || pass > 0 {
                    max_step_reference =
                        max_step_reference.max((out_reference - previous_reference).abs());
                    max_step_tabled =
                        max_step_tabled.max((out_tabled - previous_tabled).abs());
                }
                previous_reference = out_reference;
                previous_tabled = out_tabled;
            }
        }
        // Transparency bound 1e-5 (-100 dBFS; measured 7.4e-7 on this sweep).
        assert!(
            max_diff <= 1.0e-5,
            "tabled program deviation {max_diff} exceeds transparency bound"
        );
        assert!(
            max_step_tabled <= max_step_reference * 1.5,
            "tabled max step {max_step_tabled} vs reference {max_step_reference}: click?"
        );
    }

    #[test]
    fn suspended_zones_keep_phase_and_reactivate_without_clicks() {
        // AUD-06: A/B against the packaged bank. The suspended layer must skip
        // real work, keep every cursor bit-identical (phase continuity), and
        // reactivate across both crossfades (up/down), throttle swings, and
        // retention without introducing clicks. Proposed audibility criterion
        // (not an approved threshold): reactivation transient peak <= -40 dBFS
        // (0.01) and no new sample step above the reference's own maximum.
        use std::path::Path;
        let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_gf509");
        let base_config = ThreeZoneSampleLayerConfig::default();
        // fix.txt: the reference must render every zone every sample, or the
        // A/B compares identical behavior and cannot catch reactivation issues.
        let mut reference = ThreeZoneSampleLayer::load_directory(
            44_100,
            &assets,
            ThreeZoneSampleLayerConfig {
                suspend_inaudible_zones: false,
                ..base_config
            },
        )
        .unwrap();
        let mut suspended = ThreeZoneSampleLayer::load_directory(
            44_100,
            &assets,
            ThreeZoneSampleLayerConfig {
                suspend_inaudible_zones: true,
                ..base_config
            },
        )
        .unwrap();
        assert!(
            suspended.off_source_count() > 0,
            "bank must carry an off stem"
        );
        let anchors = reference.rpm_anchors();
        let rpm_lo = (anchors[0] - 2_000.0).max(1_000.0);
        let rpm_hi = anchors[2] + 1_000.0;

        // Identical drive: power sweep up, retention coast down, free-rev.
        let mut inputs = Vec::new();
        let up = 20_000usize;
        for i in 0..up {
            let u = i as f32 / (up - 1) as f32;
            inputs.push(SampleLayerInput {
                rpm: rpm_lo + (rpm_hi - rpm_lo) * u,
                throttle: 0.9,
                load: 0.85,
                normalized_engine_torque: 0.8,
                clutch_engagement: 1.0,
                crank_phase_deg: 0.0,
            });
        }
        let down = 20_000usize;
        for i in 0..down {
            let u = i as f32 / (down - 1) as f32;
            inputs.push(SampleLayerInput {
                rpm: rpm_hi + (rpm_lo - rpm_hi) * u,
                throttle: 0.0,
                load: 0.1,
                normalized_engine_torque: -0.45,
                clutch_engagement: 1.0,
                crank_phase_deg: 0.0,
            });
        }
        let free = 10_000usize;
        for i in 0..free {
            let u = i as f32 / (free - 1) as f32;
            let rpm = rpm_lo + (9_000.0 - rpm_lo) * (2.0 * u - 1.0).abs();
            inputs.push(SampleLayerInput {
                rpm,
                throttle: 0.5 * (1.0 - u),
                load: 0.0,
                normalized_engine_torque: 0.0,
                clutch_engagement: 0.0,
                crank_phase_deg: 0.0,
            });
        }

        let mut max_diff = 0.0f32;
        let mut max_step_reference = 0.0f32;
        let mut max_step_suspended = 0.0f32;
        let mut previous_reference = 0.0f32;
        let mut previous_suspended = 0.0f32;
        let mut zone_weights_seen_zero = [false; ZONE_COUNT];
        for (n, input) in inputs.iter().enumerate() {
            let out_reference = reference.process(*input).unwrap().output;
            let out_suspended = suspended.process(*input).unwrap().output;
            assert!(out_reference.is_finite() && out_suspended.is_finite(), "block {n}");
            max_diff = max_diff.max((out_reference - out_suspended).abs());
            if n > 0 {
                max_step_reference =
                    max_step_reference.max((out_reference - previous_reference).abs());
                max_step_suspended =
                    max_step_suspended.max((out_suspended - previous_suspended).abs());
            }
            previous_reference = out_reference;
            previous_suspended = out_suspended;
            let weights = multi_zone_weights(input.rpm, &anchors);
            // Legacy reference curve check on the packaged bank: the
            // generalized blend must agree with the legacy curve to within
            // the fade-window fit (a few RPM at the med/max edge).
            let legacy_anchors: [f32; ZONE_COUNT] = anchors
                .clone()
                .try_into()
                .expect("packaged bank has three anchors");
            let legacy_weights = zone_weights(
                input.rpm,
                legacy_anchors,
                base_config.max_fade_start_rpm,
                base_config.max_full_rpm,
            );
            for (seen, weight) in zone_weights_seen_zero.iter_mut().zip(weights.iter()) {
                *seen |= *weight == 0.0;
            }
            for (general, legacy) in weights.iter().zip(legacy_weights) {
                assert!(
                    (general - legacy).abs() < 0.02,
                    "rpm {}: multi {general} vs legacy {legacy}",
                    input.rpm
                );
            }
        }
        assert!(
            zone_weights_seen_zero.iter().all(|seen| *seen),
            "sweep must park every zone at zero weight at some point"
        );
        let skipped = suspended.suspended_zone_samples();
        let total_zone_renders = inputs.len() as u64 * (ZONE_COUNT as u64 + 1);
        let skip_ratio = skipped as f64 / total_zone_renders as f64;
        assert!(skipped > 0, "suspension must skip real work");
        assert!(
            skip_ratio > 0.20,
            "skip ratio {skip_ratio:.3} must clear 20% on a full-range sweep"
        );
        // Phase continuity: every cursor (zones + off stem) bit-identical.
        for index in 0..ZONE_COUNT {
            for member in 0..reference.zones[index].members.len() {
                assert_eq!(
                    reference.zones[index].members[member].zone.cursor.to_bits(),
                    suspended.zones[index].members[member].zone.cursor.to_bits(),
                    "zone {index} member {member} cursor drifted while suspended"
                );
            }
        }
        assert_eq!(
            reference
                .off_zones
                .iter()
                .map(|zone| zone.members[0].zone.cursor.to_bits())
                .collect::<Vec<_>>(),
            suspended
                .off_zones
                .iter()
                .map(|zone| zone.members[0].zone.cursor.to_bits())
                .collect::<Vec<_>>(),
            "off-stem cursor drifted while suspended"
        );
        // fix.txt measured 2026-09-05 on this sweep: transient 0.0039 with a
        // 500 rpm deep-silence margin (was 0.0120 with suspend-anywhere), skip
        // ratio 0.43. Bounds below carry ~2.5x / ~2x margin.
        assert!(
            max_diff <= 0.01,
            "reactivation transient peak {max_diff} exceeds proposed -40 dBFS criterion"
        );
        assert!(
            max_step_suspended <= max_step_reference * 1.5,
            "suspended max step {max_step_suspended} vs reference {max_step_reference}: click?"
        );
    }

    #[test]
    fn zero_weight_member_skip_is_bit_exact() {
        // The perf skip (cursor advance instead of resample for zero-weight
        // members) must not change a single output bit of the frame it
        // applies to. Single fresh frames isolate the skip: zone-level
        // suspension only affects future DSP state (pre-existing AUD-06
        // tracking policy, untouched here), never the current frame, since
        // skipped zones contribute exactly 0 either way.
        use std::path::Path;
        let bank = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_experimental");
        // Anchors, variant-span edges, midpoints, extremes, coast retention.
        let rpms = [
            4_000.0, 4_579.5, 5_000.0, 5_500.0, 6_366.0, 6_543.0, 6_720.0, 7_300.0, 7_888.5,
            8_153.0, 8_418.0, 8_443.5, 8_469.0, 8_710.5, 8_952.0, 10_000.0, 12_000.0,
            15_000.0, 18_000.0, 20_000.0,
        ];
        for rpm in rpms {
            for (throttle, load, torque) in
                [(0.95, 0.90, 0.90), (0.035, 0.10, -0.53), (0.0, 0.0, -0.2)]
            {
                let mut tracking = ThreeZoneSampleLayer::load_directory(
                    44_100,
                    &bank,
                    ThreeZoneSampleLayerConfig::default(),
                )
                .unwrap();
                let mut reference = ThreeZoneSampleLayer::load_directory(
                    44_100,
                    &bank,
                    ThreeZoneSampleLayerConfig {
                        suspend_inaudible_zones: false,
                        ..ThreeZoneSampleLayerConfig::default()
                    },
                )
                .unwrap();
                let input = SampleLayerInput {
                    rpm,
                    throttle,
                    load,
                    normalized_engine_torque: torque,
                    clutch_engagement: 1.0,
                    crank_phase_deg: 0.0,
                };
                let a = tracking.process(input).unwrap();
                let b = reference.process(input).unwrap();
                for (x, y, what) in [
                    (a.tonal, b.tonal, "tonal"),
                    (a.residual, b.residual, "residual"),
                    (a.max_rasp, b.max_rasp, "rasp"),
                    (a.off_throttle, b.off_throttle, "off"),
                    (a.output, b.output, "output"),
                ] {
                    assert_eq!(
                        x.to_bits(),
                        y.to_bits(),
                        "{what} drifted at rpm {rpm} thr {throttle}"
                    );
                }
            }
        }
        // The skip path must actually trigger on a sweep (else the test is vacuous).
        let mut layer = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        for i in 0..4_410 {
            let time_s = i as f32 / 441.0;
            let (rpm, throttle, load, torque) = if time_s < 7.0 {
                let t = time_s / 7.0;
                (5_000.0 + 13_000.0 * t * t * (3.0 - 2.0 * t), 0.95, 0.90, 0.90)
            } else {
                let t = (time_s - 7.0) / 3.0;
                (18_000.0 - 13_000.0 * (1.0 - (1.0 - t).powf(1.55)), 0.035, 0.10, -0.53)
            };
            layer
                .process(SampleLayerInput {
                    rpm,
                    throttle,
                    load,
                    normalized_engine_torque: torque,
                    clutch_engagement: 1.0,
                    crank_phase_deg: 0.0,
                })
                .unwrap();
        }
        assert!(
            layer.suspended_zone_samples() > 0,
            "skip path must actually trigger"
        );
    }

    #[test]
    fn residual_gain_scale_halves_residual_stem() {
        use std::path::Path;
        let bank = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_experimental");
        let input = SampleLayerInput {
            rpm: 12_000.0,
            throttle: 0.95,
            load: 0.9,
            normalized_engine_torque: 0.9,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        let mut full = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        let mut half = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig {
                residual_gain_scale: 0.5,
                ..ThreeZoneSampleLayerConfig::default()
            },
        )
        .unwrap();
        let loud = full.process(input).unwrap();
        let quiet = half.process(input).unwrap();
        assert!((quiet.residual - loud.residual * 0.5).abs() < 1e-6);
        assert!((quiet.tonal - loud.tonal).abs() < 1e-6, "tonal must not move");
    }

    #[test]
    fn disable_sample_rasp_zeroes_rasp_stem_and_terms() {
        // Rasp lives only on the top zone (+2.5 dB). With the mute, the stem
        // must read exactly zero and the mix must change only through the
        // removed rasp terms (finite, audible).
        use std::path::Path;
        let bank = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_experimental");
        let input = SampleLayerInput {
            rpm: 15_000.0,
            throttle: 0.95,
            load: 0.9,
            normalized_engine_torque: 0.9,
            clutch_engagement: 1.0,
            crank_phase_deg: 0.0,
        };
        let mut with_rasp = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        let mut muted = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig {
                disable_sample_rasp: true,
                ..ThreeZoneSampleLayerConfig::default()
            },
        )
        .unwrap();
        let loud = with_rasp.process(input).unwrap();
        let quiet = muted.process(input).unwrap();
        assert!(loud.output.is_finite() && quiet.output.is_finite());
        assert!(loud.max_rasp.abs() > 1e-6, "top zone must rasp at 15k rpm");
        assert_eq!(quiet.max_rasp, 0.0);
        assert!(
            (loud.output - quiet.output).abs() > 1e-6,
            "rasp mute must change the mix"
        );
    }

    #[test]
    fn experimental_schema2_bank_covers_sweep_with_all_sources() {
        // F2K integration: the prepared F2002 bank loads by manifest roles,
        // renders finite audible audio over the 3k->18k->5k evaluation
        // trajectory, carries OFF energy in the coast, and gives every one
        // of the 13 sources nonzero weight somewhere on the trajectory.
        use std::path::Path;
        let bank = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_experimental");
        let mut layer = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        assert!(!layer.legacy_three_zone, "experimental bank must use multi curves");
        assert_eq!(layer.source_labels().len(), 13);
        assert_eq!(layer.off_source_count(), 5);
        let mut accumulated = vec![0.0f32; layer.source_labels().len()];
        let mut peak = 0.0f32;
        let mut off_energy_coast = 0.0f64;
        let mut off_samples = 0u64;
        // 0-7 s accel 3000->18000 loaded, 7-10 s coast 18000->5000 lift.
        let steps = 30_000usize;
        for i in 0..steps {
            let time_s = i as f32 / steps as f32 * 10.0;
            let (rpm, throttle, load, torque) = if time_s < 7.0 {
                let t = time_s / 7.0;
                let shaped = t * t * (3.0 - 2.0 * t);
                (3_000.0 + 15_000.0 * shaped, 0.95, 0.90, 0.90)
            } else {
                let t = (time_s - 7.0) / 3.0;
                let decay = 1.0 - (1.0 - t).powf(1.55);
                let lift = (-t / 0.035).exp();
                (
                    18_000.0 - 13_000.0 * decay,
                    0.035 + (0.95 - 0.035) * lift,
                    0.10 + (0.90 - 0.10) * (-t / 0.12).exp(),
                    -0.53,
                )
            };
            let frame = layer
                .process(SampleLayerInput {
                    rpm,
                    throttle,
                    load,
                    normalized_engine_torque: torque,
                    clutch_engagement: 1.0,
                    crank_phase_deg: 0.0,
                })
                .unwrap();
            assert!(frame.output.is_finite(), "sample {i}");
            assert!(frame.tonal.is_finite() && frame.residual.is_finite());
            peak = peak.max(frame.output.abs());
            for (slot, weight) in accumulated.iter_mut().zip(frame.zone_weights.iter()) {
                *slot += weight;
            }
            if time_s >= 7.5 {
                off_energy_coast += (frame.off_throttle * frame.off_throttle) as f64;
                off_samples += 1;
            }
        }
        assert!(peak > 1e-4, "sweep must stay audible, peak {peak}");
        assert!(peak <= 1.0, "sweep must stay under PCM ceiling, peak {peak}");
        let off_rms = (off_energy_coast / off_samples as f64).sqrt();
        assert!(off_rms > 1e-5, "OFF stem must carry coast energy, rms {off_rms}");
        for (label, total) in layer.source_labels().iter().zip(accumulated.iter()) {
            assert!(
                *total > 0.0,
                "source {label} never weighted on the trajectory"
            );
        }
    }

    #[test]
    fn sampled_bank_loads_all_sources_and_covers_the_sweep() {
        // Fully-sampled V10 bank: 7 ON members in 6 zones + 4 OFF members in
        // 3 zones. The bank ships exactly the curated set that remains in
        // implementation/sfx; exterior takes and user-removed sources are
        // recorded in anchors.json:excluded. Every source keeps its measured
        // native RPM.
        use std::path::Path;
        let bank =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_sampled");
        let mut layer = ThreeZoneSampleLayer::load_directory(
            44_100,
            &bank,
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap();
        assert!(!layer.legacy_three_zone, "sampled bank must use multi curves");
        assert_eq!(layer.source_labels().len(), 11);
        assert_eq!(layer.off_source_count(), 4);
        let anchors = layer.rpm_anchors();
        assert_eq!(anchors.len(), 6, "7 members collapse into 6 ON zones");
        assert!((anchors[0] - 4_579.5).abs() < 1.0);
        assert!((anchors[anchors.len() - 1] - 8_952.0).abs() < 1.0);

        let mut accumulated = vec![0.0f32; layer.source_labels().len()];
        let mut peak = 0.0f32;
        let mut off_energy_coast = 0.0f64;
        let mut off_samples = 0u64;
        // 0-7 s accel 3000->18000 loaded, 7-10 s coast 18000->3000 lift.
        let steps = 30_000usize;
        for i in 0..steps {
            let time_s = i as f32 / steps as f32 * 10.0;
            let (rpm, throttle, load, torque) = if time_s < 7.0 {
                let t = time_s / 7.0;
                let shaped = t * t * (3.0 - 2.0 * t);
                (3_000.0 + 15_000.0 * shaped, 0.95, 0.90, 0.90)
            } else {
                let t = (time_s - 7.0) / 3.0;
                let decay = 1.0 - (1.0 - t).powf(1.55);
                let lift = (-t / 0.035).exp();
                (
                    18_000.0 - 15_000.0 * decay,
                    0.035 + (0.95 - 0.035) * lift,
                    0.10 + (0.90 - 0.10) * (-t / 0.12).exp(),
                    -0.53,
                )
            };
            let frame = layer
                .process(SampleLayerInput {
                    rpm,
                    throttle,
                    load,
                    normalized_engine_torque: torque,
                    clutch_engagement: 1.0,
                    crank_phase_deg: 0.0,
                })
                .unwrap();
            assert!(frame.output.is_finite(), "sample {i}");
            peak = peak.max(frame.output.abs());
            for (slot, weight) in accumulated.iter_mut().zip(frame.zone_weights.iter()) {
                *slot += weight;
            }
            if time_s >= 7.5 {
                off_energy_coast += (frame.off_throttle * frame.off_throttle) as f64;
                off_samples += 1;
            }
        }
        assert!(peak > 1e-4, "sweep must stay audible, peak {peak}");
        assert!(peak <= 1.0, "sweep must stay under PCM ceiling, peak {peak}");
        let off_rms = (off_energy_coast / off_samples as f64).sqrt();
        assert!(off_rms > 1e-5, "OFF stem must carry coast energy, rms {off_rms}");
        for (label, total) in layer.source_labels().iter().zip(accumulated.iter()) {
            assert!(
                *total > 0.0,
                "source {label} never weighted on the trajectory"
            );
        }
    }
}
