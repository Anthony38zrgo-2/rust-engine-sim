//! Control-thread preparation of schema-1 and schema-2 sample banks (`PreparedSampleBank`).
//! Owns all asset loading, SHA-256 verification, anchor geometry, and sinc table precomputation.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};

use crate::assets::{
    load_prepared_zone_assets, validate_asset_manifest, BankManifest, LoadedZoneAssets,
    ManifestSample,
};
use crate::config::{ThreeZoneSampleLayerConfig, ZONE_TRIM_SLOTS};
use crate::samples::crossfade::{zone_boundaries, ZONE_COUNT};
use crate::samples::resampling::{build_sinc_table, SINC_RADIUS, SINC_TABLE_PHASES};

/// Immutable prepared audio loops and anchor metadata for one zone member.
#[derive(Clone, Debug)]
pub struct PreparedMember {
    pub key: String,
    pub rpm_anchor: f32,
    pub firing_phase_deg: f32,
    pub source_sample_rate: u32,
    pub tonal: Vec<f32>,
    pub residual: Vec<f32>,
    pub variant_group: Option<String>,
    pub variant_position: usize,
    pub variant_total: usize,
}

impl PreparedMember {
    fn from_loaded(
        key: String,
        loaded: LoadedZoneAssets,
        variant_group: Option<String>,
        variant_position: usize,
        variant_total: usize,
    ) -> Self {
        Self {
            key,
            rpm_anchor: loaded.metadata.rpm_anchor,
            firing_phase_deg: loaded
                .metadata
                .firing_phase_degrees_at_loop_start
                .rem_euclid(360.0),
            source_sample_rate: loaded.metadata.sample_rate,
            tonal: loaded.tonal.samples,
            residual: loaded.residual.samples,
            variant_group,
            variant_position,
            variant_total,
        }
    }

    /// Playback ratio in source samples per output sample.
    #[inline]
    pub fn rate(&self, rpm: f32, output_sample_rate: u32) -> f64 {
        rpm as f64 / self.rpm_anchor as f64 * self.source_sample_rate as f64
            / output_sample_rate as f64
    }

    /// Computes initial loop cursor aligned to instantaneous crank phase (degrees).
    #[inline]
    pub fn aligned_cursor(&self, crank_phase_deg: f32) -> f64 {
        let target_firing_phase = (crank_phase_deg * 5.0).rem_euclid(360.0);
        let phase_delta = (target_firing_phase - self.firing_phase_deg).rem_euclid(360.0);
        let firing_cycle_samples = self.source_sample_rate as f64 * 12.0 / self.rpm_anchor as f64;
        phase_delta as f64 / 360.0 * firing_cycle_samples
    }
}

/// Immutable RPM zone grouping one or more variant members plus static DSP profile.
#[derive(Clone, Debug)]
pub struct PreparedZone {
    pub anchor: f32,
    pub members: Vec<PreparedMember>,
    pub processor_kind: usize,
    pub trim_db: f32,
    pub variant_span: (f32, f32),
}

/// Read-only, shareable prepared sample bank separated cleanly from playback state.
#[derive(Clone, Debug)]
pub struct PreparedSampleBank {
    pub output_sample_rate: u32,
    pub config: ThreeZoneSampleLayerConfig,
    pub on_zones: Vec<PreparedZone>,
    pub off_zones: Vec<PreparedZone>,
    pub on_anchors: Vec<f32>,
    pub on_bounds: Vec<(f32, f32)>,
    pub off_anchors: Vec<f32>,
    pub legacy_three_zone: bool,
    pub source_labels: Vec<String>,
    pub max_variant_group_len: usize,
    pub tabled_narrow: Vec<f32>,
}

impl PreparedSampleBank {
    /// Discovers schema version in `directory`, validates assets fail-closed, and
    /// precomputes zone geometry and windowed-sinc lookup tables.
    pub fn load_directory(
        output_sample_rate: u32,
        directory: &Path,
        config: ThreeZoneSampleLayerConfig,
    ) -> Result<Self, String> {
        let manifest_path = directory.join("manifest.json");
        if manifest_path.is_file() {
            let raw = fs::read_to_string(&manifest_path)
                .map_err(|error| format!("cannot read {}: {error}", manifest_path.display()))?;
            let peek: serde_json::Value = serde_json::from_str(&raw)
                .map_err(|error| format!("invalid {}: {error}", manifest_path.display()))?;
            let schema = peek
                .get("schema_version")
                .and_then(|value| value.as_u64())
                .unwrap_or(0);
            if schema == 2 {
                return Self::load_schema2(output_sample_rate, directory, config);
            }
        }

        let entries = fs::read_dir(directory)
            .map_err(|error| format!("cannot read {}: {error}", directory.display()))?
            .map(|entry| {
                entry
                    .map(|entry| entry.path())
                    .map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        let off_path = entries
            .iter()
            .find(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.ends_with(".off-layer.json"))
            })
            .cloned();
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
        let off_zone = off_path
            .as_deref()
            .map(load_prepared_zone_assets)
            .transpose()
            .map_err(|error| error.to_string())?;
        Self::load_with_off(output_sample_rate, metadata_paths, off_zone, config)
    }

    /// Loads three legacy `.sample-layer.json` zones without an overrun stem.
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
        off_asset: Option<LoadedZoneAssets>,
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
            .map(|path| load_prepared_zone_assets(path).map_err(|e| e.to_string()))
            .collect::<Result<Vec<_>, _>>()?;
        loaded.sort_by(|left, right| {
            left.metadata
                .rpm_anchor
                .total_cmp(&right.metadata.rpm_anchor)
        });
        if loaded
            .windows(2)
            .any(|pair| pair[1].metadata.rpm_anchor - pair[0].metadata.rpm_anchor < 100.0)
        {
            return Err("sample-layer RPM anchors must be distinct and ordered".into());
        }
        if config.max_fade_start_rpm < loaded[1].metadata.rpm_anchor
            || config.max_full_rpm > loaded[2].metadata.rpm_anchor
        {
            return Err("max-zone crossfade window must stay between med and max anchors".into());
        }
        let on_zones: Vec<PreparedZone> = loaded
            .into_iter()
            .enumerate()
            .map(|(index, asset)| {
                let key = format!("legacy-{index}");
                let anchor = asset.metadata.rpm_anchor;
                PreparedZone {
                    anchor,
                    members: vec![PreparedMember::from_loaded(key, asset, None, 0, 1)],
                    processor_kind: index,
                    trim_db: config.zone_trim_db[index.min(ZONE_TRIM_SLOTS - 1)],
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                }
            })
            .collect();
        let off_zones = off_asset
            .into_iter()
            .map(|asset| {
                let anchor = asset.metadata.rpm_anchor;
                PreparedZone {
                    anchor,
                    members: vec![PreparedMember::from_loaded(
                        "legacy-off".to_string(),
                        asset,
                        None,
                        0,
                        1,
                    )],
                    processor_kind: 1,
                    trim_db: 0.0,
                    variant_span: (f32::NEG_INFINITY, f32::INFINITY),
                }
            })
            .collect();
        Self::finalize(
            output_sample_rate,
            config,
            on_zones,
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
        let manifest = match validate_asset_manifest(directory, output_sample_rate)
            .map_err(|error| error.to_string())?
        {
            BankManifest::Schema2(manifest) => manifest,
            BankManifest::Schema1(_) => {
                return Err("expected schema-2 manifest in schema-2 loader".into());
            }
        };
        let mut on_members = load_manifest_members(directory, &manifest.samples)?;
        let mut off_members = load_manifest_members(directory, &manifest.off_samples)?;
        on_members.sort_by(|left, right| left.rpm_anchor.total_cmp(&right.rpm_anchor));
        off_members.sort_by(|left, right| left.rpm_anchor.total_cmp(&right.rpm_anchor));
        let mut on_zones = group_members(on_members)?;
        let off_zones = group_members(off_members)?;
        let last = on_zones.len() - 1;
        for (index, zone) in on_zones.iter_mut().enumerate() {
            zone.processor_kind = if index == 0 {
                0
            } else if index == last {
                2
            } else {
                1
            };
            zone.trim_db = config.zone_trim_db[index.min(ZONE_TRIM_SLOTS - 1)];
        }
        let mut labels = Vec::new();
        for zone in &on_zones {
            for member in &zone.members {
                labels.push(format!("on:{}", member.key));
            }
        }
        for zone in &off_zones {
            for member in &zone.members {
                labels.push(format!("off:{}", member.key));
            }
        }
        Self::finalize(
            output_sample_rate,
            config,
            on_zones,
            off_zones,
            false,
            labels,
        )
    }

    fn finalize(
        output_sample_rate: u32,
        config: ThreeZoneSampleLayerConfig,
        on_zones: Vec<PreparedZone>,
        off_zones: Vec<PreparedZone>,
        legacy_three_zone: bool,
        source_labels: Vec<String>,
    ) -> Result<Self, String> {
        let on_anchors: Vec<f32> = on_zones.iter().map(|zone| zone.anchor).collect();
        let on_bounds = zone_boundaries(&on_anchors);
        let off_anchors: Vec<f32> = off_zones.iter().map(|zone| zone.anchor).collect();
        let max_variant_group_len = on_zones
            .iter()
            .chain(off_zones.iter())
            .map(|zone| zone.members.len())
            .max()
            .unwrap_or(1)
            .max(1);
        Ok(Self {
            output_sample_rate,
            config,
            on_zones,
            off_zones,
            on_anchors,
            on_bounds,
            off_anchors,
            legacy_three_zone,
            source_labels,
            max_variant_group_len,
            tabled_narrow: build_sinc_table(SINC_TABLE_PHASES, SINC_RADIUS),
        })
    }
}

fn load_manifest_members(
    directory: &Path,
    entries: &[ManifestSample],
) -> Result<Vec<PreparedMember>, String> {
    let mut members = Vec::with_capacity(entries.len());
    for entry in entries {
        let loaded = load_prepared_zone_assets(&directory.join(&entry.metadata))
            .map_err(|error| error.to_string())?;
        if (loaded.metadata.rpm_anchor - entry.native_rpm).abs() > 1.0 {
            return Err(format!(
                "schema-2 entry {} anchor mismatch: manifest {} vs prepared {}",
                entry.key, entry.native_rpm, loaded.metadata.rpm_anchor
            ));
        }
        if loaded.metadata.sample_rate != entry.sample_rate
            || loaded.metadata.loop_length_samples != entry.loop_frames
        {
            return Err(format!(
                "schema-2 entry {} format mismatch against prepared metadata",
                entry.key
            ));
        }
        let (variant_group, variant_position, variant_total) = match &entry.variant {
            None => (None, 0, 1),
            Some(variant) => (Some(variant.group.clone()), variant.position, variant.count),
        };
        members.push(PreparedMember::from_loaded(
            entry.key.clone(),
            loaded,
            variant_group,
            variant_position,
            variant_total,
        ));
    }
    Ok(members)
}

fn group_members(members: Vec<PreparedMember>) -> Result<Vec<PreparedZone>, String> {
    if members.is_empty() {
        return Err("schema-2 role collection is empty".into());
    }
    {
        let mut seen: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        for (index, member) in members.iter().enumerate() {
            if let Some(group) = &member.variant_group {
                seen.entry(group.clone()).or_default().push(index);
            } else if member.variant_total != 1 || member.variant_position != 0 {
                return Err(format!(
                    "member {} has an inconsistent solo record",
                    member.key
                ));
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
            let positions: BTreeSet<usize> = indices
                .iter()
                .map(|&index| members[index].variant_position)
                .collect();
            if positions.len() != indices.len() || !positions.iter().copied().eq(0..indices.len()) {
                return Err(format!("variant group {group} positions are not 0..count"));
            }
            if indices.windows(2).any(|pair| pair[1] != pair[0] + 1) {
                return Err(format!("variant group {group} members are not contiguous"));
            }
        }
    }

    let mut zones: Vec<PreparedZone> = Vec::new();
    let mut queue: VecDeque<PreparedMember> = members.into();
    while let Some(first) = queue.pop_front() {
        if let Some(group) = first.variant_group.clone() {
            let mut run = vec![first];
            while queue
                .front()
                .is_some_and(|member| member.variant_group.as_deref() == Some(group.as_str()))
            {
                if let Some(member) = queue.pop_front() {
                    run.push(member);
                }
            }
            let anchor = run.iter().map(|member| member.rpm_anchor).sum::<f32>() / run.len() as f32;
            zones.push(PreparedZone {
                anchor,
                members: run,
                processor_kind: 1,
                trim_db: 0.0,
                variant_span: (f32::NEG_INFINITY, f32::INFINITY),
            });
        } else {
            let anchor = first.rpm_anchor;
            zones.push(PreparedZone {
                anchor,
                members: vec![first],
                processor_kind: 1,
                trim_db: 0.0,
                variant_span: (f32::NEG_INFINITY, f32::INFINITY),
            });
        }
    }

    let anchors: Vec<f32> = zones.iter().map(|zone| zone.anchor).collect();
    if anchors.windows(2).any(|pair| pair[1] - pair[0] < 1.0e-3) {
        return Err("schema-2 zone anchors must be distinct and ordered".into());
    }
    let bounds = zone_boundaries(&anchors);
    for (zone_index, zone) in zones.iter_mut().enumerate() {
        zone.variant_span = bounds[zone_index];
    }
    Ok(zones)
}
