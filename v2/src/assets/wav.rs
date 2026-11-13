//! Consolidated mono PCM16 WAV reader and writer.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::assets::validation::AssetError;

/// Decoded mono PCM16 waveform normalized by `1 / 32_768.0`.
#[derive(Clone, Debug, PartialEq)]
pub struct Pcm16Wav {
    pub sample_rate: u32,
    pub samples: Vec<f32>,
}

/// Reads and validates a mono PCM16 RIFF/WAVE file, returning normalized `[-1.0, 1.0]` samples.
pub fn read_mono_pcm16(path: &Path) -> Result<Pcm16Wav, AssetError> {
    let bytes = fs::read(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        detail: error.to_string(),
    })?;
    if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return Err(AssetError::InvalidWav(format!(
            "not a RIFF/WAVE file: {}",
            path.display()
        )));
    }
    let mut cursor = 12usize;
    let mut format = None;
    let mut data = None;
    while cursor + 8 <= bytes.len() {
        let id = &bytes[cursor..cursor + 4];
        let size = u32::from_le_bytes([
            bytes[cursor + 4],
            bytes[cursor + 5],
            bytes[cursor + 6],
            bytes[cursor + 7],
        ]) as usize;
        let start = cursor + 8;
        let end = start
            .checked_add(size)
            .filter(|&end| end <= bytes.len())
            .ok_or_else(|| {
                AssetError::InvalidWav(format!("truncated WAV chunk in {}", path.display()))
            })?;
        if id == b"fmt " && size >= 16 {
            format = Some((
                u16::from_le_bytes([bytes[start], bytes[start + 1]]),
                u16::from_le_bytes([bytes[start + 2], bytes[start + 3]]),
                u32::from_le_bytes([
                    bytes[start + 4],
                    bytes[start + 5],
                    bytes[start + 6],
                    bytes[start + 7],
                ]),
                u16::from_le_bytes([bytes[start + 14], bytes[start + 15]]),
            ));
        } else if id == b"data" {
            data = Some(&bytes[start..end]);
        }
        cursor = end + (size & 1);
    }
    let (encoding, channels, sample_rate, bits) = format.ok_or_else(|| {
        AssetError::InvalidWav(format!("missing WAV fmt chunk: {}", path.display()))
    })?;
    if encoding != 1 || channels != 1 || bits != 16 {
        return Err(AssetError::InvalidWav(format!(
            "expected mono PCM16 WAV: {}",
            path.display()
        )));
    }
    let data = data.ok_or_else(|| {
        AssetError::InvalidWav(format!("missing WAV data chunk: {}", path.display()))
    })?;
    if !data.len().is_multiple_of(2) {
        return Err(AssetError::InvalidWav(format!(
            "odd PCM16 data length: {}",
            path.display()
        )));
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

/// Writes mono PCM16 samples to `path`, rejecting non-finite samples before flushing.
pub fn write_mono_pcm16(path: &Path, sample_rate: u32, samples: &[f32]) -> Result<(), AssetError> {
    for &sample in samples {
        if !sample.is_finite() {
            return Err(AssetError::InvalidWav(
                "refusing to write a non-finite sample".into(),
            ));
        }
    }
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|error| AssetError::Io {
            path: parent.display().to_string(),
            detail: error.to_string(),
        })?;
    }
    if path.exists() {
        let _ = fs::remove_file(path);
    }
    let file = File::create(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        detail: error.to_string(),
    })?;
    let mut out = BufWriter::new(file);
    let data_bytes = (samples.len() * 2) as u32;
    let map_io = |error: std::io::Error| AssetError::Io {
        path: path.display().to_string(),
        detail: error.to_string(),
    };
    out.write_all(b"RIFF").map_err(map_io)?;
    out.write_all(&(36 + data_bytes).to_le_bytes())
        .map_err(map_io)?;
    out.write_all(b"WAVEfmt ").map_err(map_io)?;
    out.write_all(&16u32.to_le_bytes()).map_err(map_io)?;
    out.write_all(&1u16.to_le_bytes()).map_err(map_io)?;
    out.write_all(&1u16.to_le_bytes()).map_err(map_io)?;
    out.write_all(&sample_rate.to_le_bytes()).map_err(map_io)?;
    out.write_all(&(sample_rate * 2).to_le_bytes())
        .map_err(map_io)?;
    out.write_all(&2u16.to_le_bytes()).map_err(map_io)?;
    out.write_all(&16u16.to_le_bytes()).map_err(map_io)?;
    out.write_all(b"data").map_err(map_io)?;
    out.write_all(&data_bytes.to_le_bytes()).map_err(map_io)?;
    for &sample in samples {
        let pcm = (sample.clamp(-1.0, 1.0) * 32_767.0).round() as i16;
        out.write_all(&pcm.to_le_bytes()).map_err(map_io)?;
    }
    out.flush().map_err(map_io)
}
