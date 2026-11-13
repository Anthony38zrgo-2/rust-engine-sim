use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

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

#[derive(Deserialize)]
struct Schema1Entry {
    key: String,
    metadata: String,
    native_rpm: f32,
    sample_rate: u32,
    channels: u16,
    loop_frames: usize,
    tonal: String,
    tonal_sha256: String,
    residual: String,
    residual_sha256: String,
}

#[derive(Deserialize)]
struct Schema1Manifest {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<Schema1Entry>,
}

#[derive(Deserialize)]
struct Schema2Variant {
    group: String,
    position: usize,
    count: usize,
}

#[derive(Deserialize)]
struct Schema2Entry {
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
    variant: Option<Schema2Variant>,
}

#[derive(Deserialize)]
struct Schema2Manifest {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<Schema2Entry>,
    off_samples: Vec<Schema2Entry>,
}

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn sha256_file(path: &Path) -> String {
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    format!("{:x}", Sha256::digest(&bytes))
}

fn assert_single_component_name(name: &str) {
    let candidate = Path::new(name);
    assert!(
        !candidate.is_absolute() && candidate.components().count() == 1,
        "asset path must be a single local filename: {name}"
    );
}

fn parse_wav_pcm16_header(path: &Path) -> (u32, u16, usize) {
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    assert!(bytes.len() >= 44 && &bytes[0..4] == b"RIFF" && &bytes[8..12] == b"WAVE");
    let mut pos = 12usize;
    let mut sr = 0u32;
    let mut ch = 0u16;
    let mut bps = 0u16;
    let mut data_bytes = 0usize;
    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let size = u32::from_le_bytes(bytes[pos + 4..pos + 8].try_into().unwrap()) as usize;
        let payload = pos + 8;
        if id == b"fmt " && size >= 16 {
            ch = u16::from_le_bytes(bytes[payload + 2..payload + 4].try_into().unwrap());
            sr = u32::from_le_bytes(bytes[payload + 4..payload + 8].try_into().unwrap());
            bps = u16::from_le_bytes(bytes[payload + 14..payload + 16].try_into().unwrap());
        } else if id == b"data" {
            data_bytes = size;
        }
        pos = payload + size + (size % 2);
    }
    assert_eq!(bps, 16, "expected 16-bit PCM in {}", path.display());
    assert!(ch > 0 && data_bytes > 0);
    (sr, ch, data_bytes / 2 / ch as usize)
}

struct ZoneEntryRef<'a> {
    metadata: &'a str,
    native_rpm: f32,
    sample_rate: u32,
    channels: u16,
    loop_frames: usize,
    tonal: &'a str,
    tonal_sha256: &'a str,
    residual: &'a str,
    residual_sha256: &'a str,
}

fn verify_zone_entry(bank_dir: &Path, entry: &ZoneEntryRef<'_>) {
    assert_single_component_name(entry.metadata);
    assert_single_component_name(entry.tonal);
    assert_single_component_name(entry.residual);
    assert!(entry.native_rpm.is_finite() && (1_000.0..=25_000.0).contains(&entry.native_rpm));
    assert_eq!(entry.sample_rate, 48_000);
    assert_eq!(entry.channels, 1);
    assert!(entry.loop_frames >= 32);

    let meta_raw = fs::read_to_string(bank_dir.join(entry.metadata)).unwrap();
    let meta: PreparedMetadata = serde_json::from_str(&meta_raw).unwrap();
    assert_eq!(meta.schema_version, 1);
    assert_eq!(meta.sample_rate, entry.sample_rate);
    assert!((meta.rpm_anchor - entry.native_rpm).abs() < 1e-4);
    assert!(meta.firing_phase_degrees_at_loop_start.is_finite());
    assert_eq!(meta.loop_length_samples, entry.loop_frames);
    assert_eq!(meta.files.loop_tonal, entry.tonal);
    assert_eq!(meta.files.loop_residual, entry.residual);

    let tonal_path = bank_dir.join(entry.tonal);
    let residual_path = bank_dir.join(entry.residual);
    assert_eq!(sha256_file(&tonal_path), entry.tonal_sha256);
    assert_eq!(sha256_file(&residual_path), entry.residual_sha256);

    let (t_sr, t_ch, t_frames) = parse_wav_pcm16_header(&tonal_path);
    let (r_sr, r_ch, r_frames) = parse_wav_pcm16_header(&residual_path);
    assert_eq!(
        (t_sr, t_ch, t_frames),
        (entry.sample_rate, 1, entry.loop_frames)
    );
    assert_eq!(
        (r_sr, r_ch, r_frames),
        (entry.sample_rate, 1, entry.loop_frames)
    );
}

#[test]
fn fixture_bank_schema1_passes_strict_manifest_and_hash_checks() {
    let dir = fixtures_dir().join("bank_schema1");
    let raw = fs::read_to_string(dir.join("manifest.json")).expect("read schema1 manifest");
    let manifest: Schema1Manifest = serde_json::from_str(&raw).expect("parse schema1 manifest");

    assert_eq!(manifest.schema_version, 1);
    assert_eq!(manifest.key, "v10_gf509");
    assert!((manifest.output_gain - 0.61).abs() <= f32::EPSILON);
    assert_eq!(manifest.samples.len(), 3);

    for (expected_key, entry) in ["low", "med", "max"].into_iter().zip(&manifest.samples) {
        assert_eq!(entry.key, expected_key);
        verify_zone_entry(
            &dir,
            &ZoneEntryRef {
                metadata: &entry.metadata,
                native_rpm: entry.native_rpm,
                sample_rate: entry.sample_rate,
                channels: entry.channels,
                loop_frames: entry.loop_frames,
                tonal: &entry.tonal,
                tonal_sha256: &entry.tonal_sha256,
                residual: &entry.residual,
                residual_sha256: &entry.residual_sha256,
            },
        );
    }

    assert!(dir.join("overrun.off-layer.json").is_file());
    let (up_sr, up_ch, up_frames) = parse_wav_pcm16_header(&dir.join("gearup.wav"));
    let (dn_sr, dn_ch, dn_frames) = parse_wav_pcm16_header(&dir.join("geardn.wav"));
    assert_eq!((up_sr, up_ch), (48_000, 1));
    assert_eq!((dn_sr, dn_ch), (48_000, 1));
    assert!(up_frames >= 128 && dn_frames >= 128);
}

#[test]
fn fixture_bank_schema2_passes_strict_manifest_variants_and_hashes() {
    let dir = fixtures_dir().join("bank_schema2");
    let raw = fs::read_to_string(dir.join("manifest.json")).expect("read schema2 manifest");
    let manifest: Schema2Manifest = serde_json::from_str(&raw).expect("parse schema2 manifest");

    assert_eq!(manifest.schema_version, 2);
    assert_eq!(manifest.key, "v10_f2002_sampled");
    assert!((manifest.output_gain - 0.61).abs() <= f32::EPSILON);
    assert!(!manifest.samples.is_empty() && !manifest.off_samples.is_empty());

    let mut groups: BTreeMap<String, Vec<(usize, usize)>> = BTreeMap::new();
    for (expected_role, list) in [("on", &manifest.samples), ("off", &manifest.off_samples)] {
        for entry in list {
            assert_eq!(entry.role, expected_role);
            assert!(!entry.key.is_empty());
            verify_zone_entry(
                &dir,
                &ZoneEntryRef {
                    metadata: &entry.metadata,
                    native_rpm: entry.native_rpm,
                    sample_rate: entry.sample_rate,
                    channels: entry.channels,
                    loop_frames: entry.loop_frames,
                    tonal: &entry.tonal,
                    tonal_sha256: &entry.tonal_sha256,
                    residual: &entry.residual,
                    residual_sha256: &entry.residual_sha256,
                },
            );
            if let Some(variant) = &entry.variant {
                groups
                    .entry(variant.group.clone())
                    .or_default()
                    .push((variant.position, variant.count));
            }
        }
    }

    assert!(
        !groups.is_empty(),
        "schema2 fixture must exercise variant groups"
    );
    for (group, members) in &groups {
        assert!(
            members.iter().all(|(_, c)| *c == members.len()),
            "incomplete variant group {group}"
        );
        let mut positions: Vec<usize> = members.iter().map(|(p, _)| *p).collect();
        positions.sort_unstable();
        assert_eq!(positions, (0..members.len()).collect::<Vec<_>>());
    }

    let (up_sr, up_ch, up_frames) = parse_wav_pcm16_header(&dir.join("gearup.wav"));
    let (dn_sr, dn_ch, dn_frames) = parse_wav_pcm16_header(&dir.join("geardn.wav"));
    assert_eq!((up_sr, up_ch), (48_000, 1));
    assert_eq!((dn_sr, dn_ch), (48_000, 1));
    assert!(up_frames >= 128 && dn_frames >= 128);
}

struct TempBank(PathBuf);

impl TempBank {
    fn copy_from(src: &Path, label: &str) -> Self {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dst = std::env::temp_dir().join(format!("v10_v2_manifest_it_{label}_{unique}"));
        fs::create_dir_all(&dst).unwrap();
        for entry in fs::read_dir(src).unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();
            if path.is_file() {
                fs::copy(&path, dst.join(entry.file_name())).unwrap();
            }
        }
        Self(dst)
    }

    fn edit_manifest_json(&self, f: impl FnOnce(&mut serde_json::Value)) {
        let manifest_path = self.0.join("manifest.json");
        let raw = fs::read_to_string(&manifest_path).unwrap();
        let mut value: serde_json::Value = serde_json::from_str(&raw).unwrap();
        f(&mut value);
        fs::write(manifest_path, value.to_string()).unwrap();
    }
}

impl Drop for TempBank {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

#[test]
fn assets_parser_rejects_corrupt_hashes_path_traversal_and_bad_variants() {
    use v10_engine_synth::assets::{
        validate_asset_manifest, validate_bank_complete, AssetError, BankManifest,
    };

    // Valid real H0 fixtures pass both manifest validation and full WAV/metadata validation.
    let s1_src = fixtures_dir().join("bank_schema1");
    let s2_src = fixtures_dir().join("bank_schema2");
    assert!(matches!(
        validate_bank_complete(&s1_src, 48_000).unwrap(),
        BankManifest::Schema1(_)
    ));
    assert!(matches!(
        validate_bank_complete(&s2_src, 48_000).unwrap(),
        BankManifest::Schema2(_)
    ));

    // Sample-rate mismatch fails closed.
    assert!(matches!(
        validate_asset_manifest(&s1_src, 44_100),
        Err(AssetError::InvalidEntry(_))
    ));

    // Corrupt WAV byte triggers SHA-256 mismatch.
    let corrupted_hash = TempBank::copy_from(&s1_src, "corrupt_hash");
    let tonal_path = corrupted_hash.0.join("low_tonal.wav");
    let mut bytes = fs::read(&tonal_path).unwrap();
    let last = bytes.len() - 1;
    bytes[last] ^= 0x01;
    fs::write(&tonal_path, bytes).unwrap();
    assert!(matches!(
        validate_asset_manifest(&corrupted_hash.0, 48_000),
        Err(AssetError::HashMismatch { .. })
    ));

    // Path traversal inside manifest is blocked.
    let s1_traversal = TempBank::copy_from(&s1_src, "s1_traversal");
    s1_traversal.edit_manifest_json(|v| {
        v["samples"][0]["metadata"] = serde_json::json!("../escape.json");
    });
    assert!(matches!(
        validate_asset_manifest(&s1_traversal.0, 48_000),
        Err(AssetError::UnsafeLocalPath(_))
    ));

    // Wrong headroom gain in schema-2 fails header validation.
    let bad_gain = TempBank::copy_from(&s2_src, "s2_bad_gain");
    bad_gain.edit_manifest_json(|v| {
        v["output_gain"] = serde_json::json!(0.50);
    });
    assert!(matches!(
        validate_asset_manifest(&bad_gain.0, 48_000),
        Err(AssetError::InvalidManifestHeader(_))
    ));

    // Empty off_samples fails header validation.
    let no_off = TempBank::copy_from(&s2_src, "s2_no_off");
    no_off.edit_manifest_json(|v| {
        v["off_samples"] = serde_json::json!([]);
    });
    assert!(matches!(
        validate_asset_manifest(&no_off.0, 48_000),
        Err(AssetError::InvalidManifestHeader(_))
    ));

    // Incomplete or duplicated variant groups fail fail-closed.
    let bad_variant_count = TempBank::copy_from(&s2_src, "s2_bad_variant_count");
    bad_variant_count.edit_manifest_json(|v| {
        v["samples"][1]["variant"]["count"] = serde_json::json!(3);
    });
    assert!(matches!(
        validate_asset_manifest(&bad_variant_count.0, 48_000),
        Err(AssetError::InvalidVariantGroup(_))
    ));

    let bad_variant_pos = TempBank::copy_from(&s2_src, "s2_bad_variant_pos");
    bad_variant_pos.edit_manifest_json(|v| {
        v["samples"][2]["variant"]["position"] = serde_json::json!(0);
    });
    assert!(matches!(
        validate_asset_manifest(&bad_variant_pos.0, 48_000),
        Err(AssetError::InvalidVariantGroup(_))
    ));

    // Duplicate keys are rejected rather than misclassified by role.
    let dup_key = TempBank::copy_from(&s2_src, "s2_dup_key");
    dup_key.edit_manifest_json(|v| {
        v["off_samples"][0]["key"] = serde_json::json!("on_low");
    });
    assert!(matches!(
        validate_asset_manifest(&dup_key.0, 48_000),
        Err(AssetError::InvalidEntry(_))
    ));

    // Prepared metadata anchor mismatch against manifest native_rpm fails complete validation.
    let anchor_mismatch = TempBank::copy_from(&s2_src, "s2_anchor_mismatch");
    let meta_path = anchor_mismatch.0.join("on_low.json");
    let mut meta_val: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&meta_path).unwrap()).unwrap();
    meta_val["rpm_anchor"] = serde_json::json!(9_999.0);
    fs::write(&meta_path, meta_val.to_string()).unwrap();
    assert!(matches!(
        validate_bank_complete(&anchor_mismatch.0, 48_000),
        Err(AssetError::InvalidMetadata(_))
    ));

    // Non-contiguous variant anchors fail complete bank validation.
    let non_contiguous = TempBank::copy_from(&s2_src, "s2_non_contiguous_variant");
    let meta_mid_b = non_contiguous.0.join("on_mid_b.json");
    let mut mid_b_val: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&meta_mid_b).unwrap()).unwrap();
    mid_b_val["rpm_anchor"] = serde_json::json!(15_000.0);
    fs::write(&meta_mid_b, mid_b_val.to_string()).unwrap();
    non_contiguous.edit_manifest_json(|v| {
        v["samples"][2]["native_rpm"] = serde_json::json!(15_000.0);
    });
    assert!(matches!(
        validate_bank_complete(&non_contiguous.0, 48_000),
        Err(AssetError::InvalidVariantGroup(_))
    ));
}
