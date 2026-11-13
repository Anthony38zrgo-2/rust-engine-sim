//! Fail-closed asset validation primitives: typed errors, local filename traversal
//! guards, and SHA-256 verification.

use std::fmt;
use std::fs;
use std::path::{Component, Path};

use sha2::{Digest, Sha256};

/// Frozen output headroom multiplier required by every GF509/F2002 bank manifest.
pub const GF509_HEADROOM_GAIN: f32 = 0.61;

/// Structured error returned by all asset parsing, WAV decoding, and fail-closed
/// manifest verification routines.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AssetError {
    Io { path: String, detail: String },
    InvalidJson { path: String, detail: String },
    UnsafeLocalPath(String),
    MissingFile(String),
    HashMismatch { path: String },
    InvalidManifestHeader(String),
    InvalidEntry(String),
    InvalidVariantGroup(String),
    InvalidMetadata(String),
    InvalidWav(String),
}

impl fmt::Display for AssetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, detail } => write!(f, "asset I/O error at {path}: {detail}"),
            Self::InvalidJson { path, detail } => write!(f, "invalid JSON at {path}: {detail}"),
            Self::UnsafeLocalPath(name) => {
                write!(f, "asset path must be a single local filename: {name}")
            }
            Self::MissingFile(name) => write!(f, "missing asset file: {name}"),
            Self::HashMismatch { path } => write!(f, "asset SHA-256 mismatch: {path}"),
            Self::InvalidManifestHeader(msg) => write!(f, "invalid manifest header: {msg}"),
            Self::InvalidEntry(msg) => write!(f, "invalid manifest entry: {msg}"),
            Self::InvalidVariantGroup(msg) => write!(f, "invalid variant group: {msg}"),
            Self::InvalidMetadata(msg) => write!(f, "invalid prepared metadata: {msg}"),
            Self::InvalidWav(msg) => write!(f, "invalid WAV asset: {msg}"),
        }
    }
}

impl std::error::Error for AssetError {}

impl From<AssetError> for String {
    fn from(error: AssetError) -> Self {
        error.to_string()
    }
}

/// Rejects empty strings, drive/root prefixes, parent (`..`) traversal, or any
/// nested path component; only single-segment local filenames are permitted.
pub fn ensure_local_filename(name: &str) -> Result<&Path, AssetError> {
    if name.is_empty() || name.contains('\\') {
        return Err(AssetError::UnsafeLocalPath(name.to_string()));
    }
    let candidate = Path::new(name);
    if candidate.is_absolute() {
        return Err(AssetError::UnsafeLocalPath(name.to_string()));
    }
    let mut components = candidate.components();
    match (components.next(), components.next()) {
        (Some(Component::Normal(_)), None) => Ok(candidate),
        _ => Err(AssetError::UnsafeLocalPath(name.to_string())),
    }
}

/// Computes lowercase hex SHA-256 digest of an in-memory buffer.
pub fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

/// Reads `path` and verifies its lowercase hex SHA-256 matches `expected_hex`.
pub fn verify_file_sha256(path: &Path, expected_hex: &str) -> Result<(), AssetError> {
    if !path.is_file() {
        return Err(AssetError::MissingFile(path.display().to_string()));
    }
    let bytes = fs::read(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        detail: error.to_string(),
    })?;
    let actual = sha256_hex(&bytes);
    if actual != expected_hex {
        return Err(AssetError::HashMismatch {
            path: path.display().to_string(),
        });
    }
    Ok(())
}
