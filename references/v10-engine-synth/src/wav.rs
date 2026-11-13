use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

pub fn write_mono_pcm16(path: &Path, sample_rate: u32, samples: &[f32]) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    if path.exists() {
        let _ = std::fs::remove_file(path);
    }
    let mut out = BufWriter::new(File::create(path).map_err(|e| e.to_string())?);
    let data_bytes = (samples.len() * 2) as u32;
    out.write_all(b"RIFF").map_err(|e| e.to_string())?;
    out.write_all(&(36 + data_bytes).to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(b"WAVEfmt ").map_err(|e| e.to_string())?;
    out.write_all(&16u32.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&1u16.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&1u16.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&sample_rate.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&(sample_rate * 2).to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&2u16.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(&16u16.to_le_bytes())
        .map_err(|e| e.to_string())?;
    out.write_all(b"data").map_err(|e| e.to_string())?;
    out.write_all(&data_bytes.to_le_bytes())
        .map_err(|e| e.to_string())?;
    for &sample in samples {
        if !sample.is_finite() {
            return Err("refusing to write a non-finite sample".into());
        }
        let pcm = (sample.clamp(-1.0, 1.0) * 32_767.0).round() as i16;
        out.write_all(&pcm.to_le_bytes())
            .map_err(|e| e.to_string())?;
    }
    out.flush().map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_mono_pcm16_writes_header_and_rejects_non_finite() {
        let dir = std::env::temp_dir().join(format!("f90_wav_test_{}", std::process::id()));
        let path = dir.join("mono.wav");
        write_mono_pcm16(&path, 48_000, &[0.0, 0.5, -0.5, 1.0]).expect("write wav");
        let bytes = std::fs::read(&path).expect("read wav");
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        assert_eq!(&bytes[36..40], b"data");
        assert_eq!(bytes.len(), 44 + 4 * 2, "44-byte header + 4 i16 samples");
        assert!(write_mono_pcm16(&path, 48_000, &[f32::NAN]).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
