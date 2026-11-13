//! Asset loading, mono PCM16 WAV codecs, and fail-closed manifest validation.

pub mod manifest;
pub mod validation;
pub mod wav;

pub use manifest::{
    load_prepared_metadata, load_prepared_zone_assets, validate_asset_manifest,
    validate_bank_complete, BankManifest, LoadedZoneAssets, ManifestSample, ManifestSchema2,
    ManifestVariant, PreparedFiles, PreparedMetadata, Schema1Entry, Schema1Manifest,
};
pub use validation::{
    ensure_local_filename, sha256_hex, verify_file_sha256, AssetError, GF509_HEADROOM_GAIN,
};
pub use wav::{read_mono_pcm16, write_mono_pcm16, Pcm16Wav};

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::path::{Path, PathBuf};

    struct TempDirGuard(PathBuf);
    impl Drop for TempDirGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn temp_dir(label: &str) -> TempDirGuard {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("v10_v2_assets_{label}_{unique}"));
        std::fs::create_dir_all(&dir).unwrap();
        TempDirGuard(dir)
    }

    fn write_bytes(dir: &Path, name: &str, bytes: &[u8]) -> String {
        std::fs::write(dir.join(name), bytes).unwrap();
        sha256_hex(bytes)
    }

    fn experimental_entry(
        dir: &Path,
        key: &str,
        role: &str,
        sample_rate: u32,
    ) -> serde_json::Value {
        let metadata = format!("{key}_meta.json");
        let tonal = format!("{key}_tonal.pcm");
        let residual = format!("{key}_residual.pcm");
        write_bytes(dir, &metadata, b"{}");
        let tonal_sha = write_bytes(dir, &tonal, format!("{key}-tonal").as_bytes());
        let residual_sha = write_bytes(dir, &residual, format!("{key}-residual").as_bytes());
        json!({
            "key": key,
            "role": role,
            "metadata": metadata,
            "native_rpm": 8000.0,
            "sample_rate": sample_rate,
            "channels": 1,
            "loop_frames": 128,
            "tonal": tonal,
            "tonal_sha256": tonal_sha,
            "residual": residual,
            "residual_sha256": residual_sha,
        })
    }

    fn experimental_manifest_value(dir: &Path, sample_rate: u32) -> serde_json::Value {
        json!({
            "schema_version": 2,
            "key": "v10_f2002_experimental",
            "output_gain": GF509_HEADROOM_GAIN,
            "samples": [experimental_entry(dir, "on_low", "on", sample_rate)],
            "off_samples": [experimental_entry(dir, "off_low", "off", sample_rate)],
        })
    }

    fn validate_value(
        dir: &Path,
        sample_rate: u32,
        value: &serde_json::Value,
    ) -> Result<BankManifest, AssetError> {
        std::fs::write(dir.join("manifest.json"), value.to_string()).unwrap();
        validate_asset_manifest(dir, sample_rate)
    }

    #[test]
    fn write_and_read_mono_pcm16_roundtrips_and_rejects_non_finite() {
        let guard = temp_dir("wav");
        let path = guard.0.join("mono.wav");
        write_mono_pcm16(&path, 48_000, &[0.0, 0.5, -0.5, 1.0]).expect("write wav");
        let bytes = std::fs::read(&path).expect("read wav");
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        assert_eq!(&bytes[36..40], b"data");
        assert_eq!(bytes.len(), 44 + 4 * 2);

        let decoded = read_mono_pcm16(&path).expect("read decoded wav");
        assert_eq!(decoded.sample_rate, 48_000);
        assert_eq!(decoded.samples.len(), 4);
        assert!((decoded.samples[1] - 0.5).abs() < 1.0e-4);
        assert!((decoded.samples[2] + 0.5).abs() < 1.0e-4);
        assert!((decoded.samples[3] - 32_767.0 / 32_768.0).abs() < 1.0e-6);

        assert!(matches!(
            write_mono_pcm16(&path, 48_000, &[f32::NAN]),
            Err(AssetError::InvalidWav(_))
        ));
    }

    #[test]
    fn ensure_local_filename_blocks_traversal_and_nested_paths() {
        assert!(ensure_local_filename("low_tonal.wav").is_ok());
        assert!(matches!(
            ensure_local_filename("../escape.json"),
            Err(AssetError::UnsafeLocalPath(_))
        ));
        assert!(matches!(
            ensure_local_filename("sub/file.wav"),
            Err(AssetError::UnsafeLocalPath(_))
        ));
        assert!(matches!(
            ensure_local_filename(r"sub\file.wav"),
            Err(AssetError::UnsafeLocalPath(_))
        ));
        assert!(matches!(
            ensure_local_filename(""),
            Err(AssetError::UnsafeLocalPath(_))
        ));
    }

    #[test]
    fn experimental_manifest_accepts_complete_bank_and_rejects_defects() {
        let guard = temp_dir("manifest");
        let dir = &guard.0;
        let valid = experimental_manifest_value(dir, 44_100);
        assert!(validate_value(dir, 44_100, &valid).is_ok());

        let mut bad = valid.clone();
        bad["key"] = json!("wrong");
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidManifestHeader(_))
        ));

        let mut bad = valid.clone();
        bad["output_gain"] = json!(0.5);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidManifestHeader(_))
        ));

        let mut bad = valid.clone();
        bad["samples"] = json!([]);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidManifestHeader(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["role"] = json!("off");
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidEntry(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["native_rpm"] = json!(1e40);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidEntry(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["sample_rate"] = json!(22_050);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidEntry(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["channels"] = json!(2);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidEntry(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["loop_frames"] = json!(8);
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::InvalidEntry(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["metadata"] = json!("../escape.json");
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::UnsafeLocalPath(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["metadata"] = json!("absent.json");
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::MissingFile(_))
        ));

        let mut bad = valid.clone();
        bad["samples"][0]["tonal_sha256"] = json!("00");
        assert!(matches!(
            validate_value(dir, 44_100, &bad),
            Err(AssetError::HashMismatch { .. })
        ));
    }

    #[test]
    fn experimental_manifest_variants_must_be_complete_and_ordered() {
        let guard = temp_dir("variants");
        let dir = &guard.0;

        let mut value = experimental_manifest_value(dir, 44_100);
        value["samples"][0]["variant"] = json!({"group": "rpm", "position": 0, "count": 2});
        let mut second = experimental_entry(dir, "on_high", "on", 44_100);
        second["variant"] = json!({"group": "rpm", "position": 1, "count": 2});
        value["samples"].as_array_mut().unwrap().push(second);
        assert!(validate_value(dir, 44_100, &value).is_ok());

        let mut incomplete = value.clone();
        incomplete["samples"][0]["variant"]["count"] = json!(3);
        assert!(matches!(
            validate_value(dir, 44_100, &incomplete),
            Err(AssetError::InvalidVariantGroup(_))
        ));

        let mut duplicate = value.clone();
        duplicate["samples"][1]["variant"]["position"] = json!(0);
        assert!(matches!(
            validate_value(dir, 44_100, &duplicate),
            Err(AssetError::InvalidVariantGroup(_))
        ));

        let mut bad_record = value.clone();
        bad_record["samples"][0]["variant"]["group"] = json!("");
        assert!(matches!(
            validate_value(dir, 44_100, &bad_record),
            Err(AssetError::InvalidVariantGroup(_))
        ));

        let mut bad_record = value.clone();
        bad_record["samples"][0]["variant"]["count"] = json!(1);
        assert!(matches!(
            validate_value(dir, 44_100, &bad_record),
            Err(AssetError::InvalidVariantGroup(_))
        ));
    }
}
