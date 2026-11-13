//! Fail-closed manifest (schema 1 & schema 2) and prepared-zone asset parsing.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

use serde::Deserialize;

use crate::assets::validation::{
    ensure_local_filename, verify_file_sha256, AssetError, GF509_HEADROOM_GAIN,
};
use crate::assets::wav::{read_mono_pcm16, Pcm16Wav};

/// File references inside a prepared zone metadata JSON (`*.sample-layer.json` or `*.json`).
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
pub struct PreparedFiles {
    pub loop_tonal: String,
    pub loop_residual: String,
}

/// Per-zone prepared metadata describing loop length, phase anchor, and stems.
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct PreparedMetadata {
    pub schema_version: u32,
    pub sample_rate: u32,
    pub rpm_anchor: f32,
    pub firing_phase_degrees_at_loop_start: f32,
    pub loop_length_samples: usize,
    pub files: PreparedFiles,
}

/// Loaded metadata and decoded mono PCM16 tonal/residual loop buffers for one zone.
#[derive(Clone, Debug)]
pub struct LoadedZoneAssets {
    pub metadata: PreparedMetadata,
    pub tonal: Pcm16Wav,
    pub residual: Pcm16Wav,
}

/// Single entry in a schema-1 (`v10_gf509`) sample manifest.
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct Schema1Entry {
    pub key: String,
    pub metadata: String,
    pub native_rpm: f32,
    pub sample_rate: u32,
    pub channels: u16,
    pub loop_frames: usize,
    pub tonal: String,
    pub tonal_sha256: String,
    pub residual: String,
    pub residual_sha256: String,
}

/// Legacy schema-1 (`v10_gf509`) 3-zone manifest.
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct Schema1Manifest {
    pub schema_version: u32,
    pub key: String,
    pub output_gain: f32,
    pub samples: Vec<Schema1Entry>,
}

/// Multi-variant RPM group annotation inside schema-2 manifests.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
pub struct ManifestVariant {
    pub group: String,
    pub position: usize,
    pub count: usize,
}

/// Single ON or OFF sample entry in a schema-2 manifest.
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct ManifestSample {
    pub key: String,
    pub role: String,
    pub metadata: String,
    pub native_rpm: f32,
    pub sample_rate: u32,
    pub channels: u16,
    pub loop_frames: usize,
    pub tonal: String,
    pub tonal_sha256: String,
    pub residual: String,
    pub residual_sha256: String,
    pub variant: Option<ManifestVariant>,
}

/// Schema-2 (`v10_f2002_experimental` / `v10_f2002_sampled`) bank manifest.
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct ManifestSchema2 {
    pub schema_version: u32,
    pub key: String,
    pub output_gain: f32,
    pub samples: Vec<ManifestSample>,
    pub off_samples: Vec<ManifestSample>,
}

/// Validated asset bank manifest (schema 1 or schema 2).
#[derive(Clone, Debug, PartialEq)]
pub enum BankManifest {
    Schema1(Schema1Manifest),
    Schema2(ManifestSchema2),
}

/// Parses and validates a bank `manifest.json` in `directory` against
/// `output_sample_rate`. Checks headers, local filename safety, file existence,
/// SHA-256 digests, and complete `0..count` variant groups.
pub fn validate_asset_manifest(
    directory: &Path,
    output_sample_rate: u32,
) -> Result<BankManifest, AssetError> {
    let path = directory.join("manifest.json");
    let raw = fs::read_to_string(&path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        detail: error.to_string(),
    })?;
    let peek: serde_json::Value =
        serde_json::from_str(&raw).map_err(|error| AssetError::InvalidJson {
            path: path.display().to_string(),
            detail: error.to_string(),
        })?;
    let schema = peek
        .get("schema_version")
        .and_then(|value| value.as_u64())
        .unwrap_or(0);
    match schema {
        1 => validate_schema1_manifest(directory, output_sample_rate, &path, &raw)
            .map(BankManifest::Schema1),
        2 => validate_schema2_manifest(directory, output_sample_rate, &path, &raw)
            .map(BankManifest::Schema2),
        other => Err(AssetError::InvalidManifestHeader(format!(
            "unsupported manifest schema_version {other}"
        ))),
    }
}

/// Validates the manifest AND loads/verifies every referenced prepared metadata +
/// PCM16 WAV file (including anchor tolerance, loop frame counts, and contiguous
/// anchor-ordered variant runs).
pub fn validate_bank_complete(
    directory: &Path,
    output_sample_rate: u32,
) -> Result<BankManifest, AssetError> {
    let manifest = validate_asset_manifest(directory, output_sample_rate)?;
    match &manifest {
        BankManifest::Schema1(s1) => {
            for entry in &s1.samples {
                verify_entry_prepared_assets(
                    directory,
                    &entry.key,
                    &entry.metadata,
                    entry.native_rpm,
                    entry.sample_rate,
                    entry.loop_frames,
                )?;
            }
        }
        BankManifest::Schema2(s2) => {
            verify_schema2_collection_assets(directory, &s2.samples)?;
            verify_schema2_collection_assets(directory, &s2.off_samples)?;
        }
    }
    Ok(manifest)
}

/// Loads and validates a single prepared metadata JSON and its mono PCM16 WAV loops.
pub fn load_prepared_zone_assets(metadata_path: &Path) -> Result<LoadedZoneAssets, AssetError> {
    let metadata = load_prepared_metadata(metadata_path)?;
    let parent = metadata_path.parent().unwrap_or_else(|| Path::new("."));
    let tonal_rel = ensure_local_filename(&metadata.files.loop_tonal)?;
    let residual_rel = ensure_local_filename(&metadata.files.loop_residual)?;
    let tonal = read_mono_pcm16(&parent.join(tonal_rel))?;
    let residual = read_mono_pcm16(&parent.join(residual_rel))?;
    if tonal.sample_rate != metadata.sample_rate || residual.sample_rate != metadata.sample_rate {
        return Err(AssetError::InvalidMetadata(format!(
            "WAV/metadata sample-rate mismatch in {}",
            metadata_path.display()
        )));
    }
    if tonal.samples.len() != metadata.loop_length_samples
        || residual.samples.len() != metadata.loop_length_samples
        || tonal.samples.len() < 32
    {
        return Err(AssetError::InvalidMetadata(format!(
            "WAV/metadata loop-length mismatch in {}",
            metadata_path.display()
        )));
    }
    Ok(LoadedZoneAssets {
        metadata,
        tonal,
        residual,
    })
}

/// Parses and validates a `PreparedMetadata` JSON file without reading the audio buffers.
pub fn load_prepared_metadata(metadata_path: &Path) -> Result<PreparedMetadata, AssetError> {
    let raw = fs::read_to_string(metadata_path).map_err(|error| AssetError::Io {
        path: metadata_path.display().to_string(),
        detail: error.to_string(),
    })?;
    let metadata: PreparedMetadata =
        serde_json::from_str(&raw).map_err(|error| AssetError::InvalidJson {
            path: metadata_path.display().to_string(),
            detail: error.to_string(),
        })?;
    if metadata.schema_version != 1 {
        return Err(AssetError::InvalidMetadata(format!(
            "unsupported prepared metadata schema {} in {}",
            metadata.schema_version,
            metadata_path.display()
        )));
    }
    if !(8_000..=192_000).contains(&metadata.sample_rate) {
        return Err(AssetError::InvalidMetadata(format!(
            "sample_rate out of range in {}: {}",
            metadata_path.display(),
            metadata.sample_rate
        )));
    }
    if !metadata.rpm_anchor.is_finite() || !(1_000.0..=25_000.0).contains(&metadata.rpm_anchor) {
        return Err(AssetError::InvalidMetadata(format!(
            "invalid rpm_anchor in {}",
            metadata_path.display()
        )));
    }
    if !metadata.firing_phase_degrees_at_loop_start.is_finite() {
        return Err(AssetError::InvalidMetadata(format!(
            "invalid firing phase in {}",
            metadata_path.display()
        )));
    }
    if metadata.loop_length_samples < 32 {
        return Err(AssetError::InvalidMetadata(format!(
            "loop_length_samples < 32 in {}",
            metadata_path.display()
        )));
    }
    ensure_local_filename(&metadata.files.loop_tonal)?;
    ensure_local_filename(&metadata.files.loop_residual)?;
    Ok(metadata)
}

fn validate_schema1_manifest(
    directory: &Path,
    output_sample_rate: u32,
    manifest_path: &Path,
    raw: &str,
) -> Result<Schema1Manifest, AssetError> {
    let manifest: Schema1Manifest =
        serde_json::from_str(raw).map_err(|error| AssetError::InvalidJson {
            path: manifest_path.display().to_string(),
            detail: error.to_string(),
        })?;
    if manifest.schema_version != 1 || manifest.key != "v10_gf509" {
        return Err(AssetError::InvalidManifestHeader(
            "GF509 schema-1 manifest schema/key mismatch".into(),
        ));
    }
    if !manifest.output_gain.is_finite()
        || (manifest.output_gain - GF509_HEADROOM_GAIN).abs() > f32::EPSILON
    {
        return Err(AssetError::InvalidManifestHeader(
            "GF509 manifest output gain does not match 0.61".into(),
        ));
    }
    if manifest.samples.len() != 3 {
        return Err(AssetError::InvalidManifestHeader(format!(
            "GF509 manifest expected 3 samples, found {}",
            manifest.samples.len()
        )));
    }
    for (expected_key, entry) in ["low", "med", "max"].into_iter().zip(&manifest.samples) {
        if entry.key != expected_key
            || !entry.native_rpm.is_finite()
            || !(1_000.0..=25_000.0).contains(&entry.native_rpm)
            || entry.sample_rate != output_sample_rate
            || entry.channels != 1
            || entry.loop_frames < 32
        {
            return Err(AssetError::InvalidEntry(format!(
                "invalid GF509 manifest entry: {}",
                entry.key
            )));
        }
        verify_local_and_hashes(
            directory,
            &entry.metadata,
            &entry.tonal,
            &entry.tonal_sha256,
            &entry.residual,
            &entry.residual_sha256,
        )?;
    }
    Ok(manifest)
}

fn validate_schema2_manifest(
    directory: &Path,
    output_sample_rate: u32,
    manifest_path: &Path,
    raw: &str,
) -> Result<ManifestSchema2, AssetError> {
    let manifest: ManifestSchema2 =
        serde_json::from_str(raw).map_err(|error| AssetError::InvalidJson {
            path: manifest_path.display().to_string(),
            detail: error.to_string(),
        })?;
    if manifest.schema_version != 2
        || !matches!(
            manifest.key.as_str(),
            "v10_f2002_experimental" | "v10_f2002_sampled"
        )
    {
        return Err(AssetError::InvalidManifestHeader(
            "schema-2 manifest schema/key mismatch".into(),
        ));
    }
    if !manifest.output_gain.is_finite()
        || (manifest.output_gain - GF509_HEADROOM_GAIN).abs() > f32::EPSILON
    {
        return Err(AssetError::InvalidManifestHeader(
            "schema-2 manifest output gain does not match 0.61".into(),
        ));
    }
    if manifest.samples.is_empty() || manifest.off_samples.is_empty() {
        return Err(AssetError::InvalidManifestHeader(
            "schema-2 manifest requires non-empty samples and off_samples".into(),
        ));
    }

    let mut seen_keys = BTreeSet::new();
    for entry in &manifest.samples {
        if !seen_keys.insert(entry.key.as_str()) {
            return Err(AssetError::InvalidEntry(format!(
                "duplicate manifest key: {}",
                entry.key
            )));
        }
        validate_schema2_entry(entry, directory, output_sample_rate, "on")?;
    }
    for entry in &manifest.off_samples {
        if !seen_keys.insert(entry.key.as_str()) {
            return Err(AssetError::InvalidEntry(format!(
                "duplicate manifest key: {}",
                entry.key
            )));
        }
        validate_schema2_entry(entry, directory, output_sample_rate, "off")?;
    }
    validate_schema2_variants(&manifest.samples)?;
    validate_schema2_variants(&manifest.off_samples)?;
    Ok(manifest)
}

fn validate_schema2_entry(
    entry: &ManifestSample,
    directory: &Path,
    output_sample_rate: u32,
    expected_role: &str,
) -> Result<(), AssetError> {
    if entry.key.is_empty()
        || entry.role != expected_role
        || !entry.native_rpm.is_finite()
        || !(1_000.0..=25_000.0).contains(&entry.native_rpm)
        || entry.sample_rate != output_sample_rate
        || entry.channels != 1
        || entry.loop_frames < 32
    {
        return Err(AssetError::InvalidEntry(format!(
            "invalid schema-2 entry: {}",
            entry.key
        )));
    }
    verify_local_and_hashes(
        directory,
        &entry.metadata,
        &entry.tonal,
        &entry.tonal_sha256,
        &entry.residual,
        &entry.residual_sha256,
    )?;
    if let Some(variant) = &entry.variant {
        if variant.count < 2 || variant.position >= variant.count || variant.group.is_empty() {
            return Err(AssetError::InvalidVariantGroup(format!(
                "invalid variant record for entry: {}",
                entry.key
            )));
        }
    }
    Ok(())
}

fn verify_local_and_hashes(
    directory: &Path,
    metadata: &str,
    tonal: &str,
    tonal_sha256: &str,
    residual: &str,
    residual_sha256: &str,
) -> Result<(), AssetError> {
    for name in [metadata, tonal, residual] {
        let local = ensure_local_filename(name)?;
        if !directory.join(local).is_file() {
            return Err(AssetError::MissingFile(name.to_string()));
        }
    }
    verify_file_sha256(&directory.join(tonal), tonal_sha256)?;
    verify_file_sha256(&directory.join(residual), residual_sha256)?;
    Ok(())
}

fn validate_schema2_variants(collection: &[ManifestSample]) -> Result<(), AssetError> {
    let mut groups: BTreeMap<&str, Vec<(&str, usize, usize)>> = BTreeMap::new();
    for entry in collection {
        if let Some(variant) = &entry.variant {
            groups.entry(variant.group.as_str()).or_default().push((
                entry.key.as_str(),
                variant.position,
                variant.count,
            ));
        }
    }
    for (group, members) in &groups {
        if members.iter().any(|(_, _, count)| *count != members.len()) {
            return Err(AssetError::InvalidVariantGroup(format!(
                "variant group {group} is incomplete"
            )));
        }
        let mut positions: Vec<usize> = members.iter().map(|(_, position, _)| *position).collect();
        positions.sort_unstable();
        if positions != (0..members.len()).collect::<Vec<_>>() {
            return Err(AssetError::InvalidVariantGroup(format!(
                "variant group {group} positions invalid"
            )));
        }
    }
    Ok(())
}

fn verify_entry_prepared_assets(
    directory: &Path,
    key: &str,
    metadata_file: &str,
    native_rpm: f32,
    sample_rate: u32,
    loop_frames: usize,
) -> Result<(), AssetError> {
    let loaded = load_prepared_zone_assets(&directory.join(metadata_file))?;
    if (loaded.metadata.rpm_anchor - native_rpm).abs() > 1.0 {
        return Err(AssetError::InvalidMetadata(format!(
            "entry {key} anchor mismatch: manifest {native_rpm} vs prepared {}",
            loaded.metadata.rpm_anchor
        )));
    }
    if loaded.metadata.sample_rate != sample_rate
        || loaded.metadata.loop_length_samples != loop_frames
    {
        return Err(AssetError::InvalidMetadata(format!(
            "entry {key} format mismatch between manifest and prepared metadata"
        )));
    }
    Ok(())
}

fn verify_schema2_collection_assets(
    directory: &Path,
    entries: &[ManifestSample],
) -> Result<(), AssetError> {
    let mut sorted: Vec<&ManifestSample> = entries.iter().collect();
    sorted.sort_by(|left, right| left.native_rpm.total_cmp(&right.native_rpm));
    for entry in &sorted {
        verify_entry_prepared_assets(
            directory,
            &entry.key,
            &entry.metadata,
            entry.native_rpm,
            entry.sample_rate,
            entry.loop_frames,
        )?;
    }
    let mut finished_groups = BTreeSet::new();
    let mut active_group: Option<&str> = None;
    for entry in sorted {
        match &entry.variant {
            Some(variant) => {
                if active_group != Some(variant.group.as_str()) {
                    if let Some(previous) = active_group {
                        finished_groups.insert(previous);
                    }
                    if finished_groups.contains(variant.group.as_str()) {
                        return Err(AssetError::InvalidVariantGroup(format!(
                            "variant group {} is not contiguous by RPM anchor",
                            variant.group
                        )));
                    }
                    active_group = Some(variant.group.as_str());
                }
            }
            None => {
                if let Some(previous) = active_group.take() {
                    finished_groups.insert(previous);
                }
            }
        }
    }
    Ok(())
}
