//! WAV file read/write using `hound`.

use hound::{SampleFormat, WavReader, WavSpec, WavWriter};
use std::path::Path;

// ---------------------------------------------------------------------------
// Write
// ---------------------------------------------------------------------------

/// Write mono i16 samples to a WAV file.
pub fn write_wav_i16_mono<P: AsRef<Path>>(path: P, sample_rate: u32, samples: &[i16]) -> Result<(), String> {
    let spec = WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: SampleFormat::Int,
    };
    let mut writer = WavWriter::create(path.as_ref(), spec).map_err(|e| e.to_string())?;
    for &s in samples {
        writer.write_sample(s).map_err(|e| e.to_string())?;
    }
    writer.finalize().map_err(|e| e.to_string())
}

/// Write multi-channel i16 samples (interleaved) to a WAV file.
pub fn write_wav_i16_interleaved<P: AsRef<Path>>(
    path: P,
    sample_rate: u32,
    channels: u16,
    samples: &[i16],
) -> Result<(), String> {
    let spec = WavSpec {
        channels,
        sample_rate,
        bits_per_sample: 16,
        sample_format: SampleFormat::Int,
    };
    let mut writer = WavWriter::create(path.as_ref(), spec).map_err(|e| e.to_string())?;
    for &s in samples {
        writer.write_sample(s).map_err(|e| e.to_string())?;
    }
    writer.finalize().map_err(|e| e.to_string())
}

/// Write mono f64 samples (normalized [-1,1]) as 16-bit WAV.
pub fn write_wav_f64_mono<P: AsRef<Path>>(path: P, sample_rate: u32, samples: &[f64]) -> Result<(), String> {
    let i16s: Vec<i16> = samples
        .iter()
        .map(|&s| (s.clamp(-1.0, 1.0) * i16::MAX as f64).round() as i16)
        .collect();
    write_wav_i16_mono(path, sample_rate, &i16s)
}

// ---------------------------------------------------------------------------
// Read
// ---------------------------------------------------------------------------

/// Read a WAV file and return (sample_rate, mono i16 samples).
/// Multi-channel input is downmixed to mono by averaging.
pub fn read_wav_mono_i16<P: AsRef<Path>>(path: P) -> Result<(u32, Vec<i16>), String> {
    let mut reader = WavReader::open(path.as_ref()).map_err(|e| e.to_string())?;
    let spec = reader.spec();
    let sample_rate = spec.sample_rate;
    let channels = spec.channels as usize;

    let samples: Result<Vec<i16>, _> = reader.samples::<i16>().collect();
    let samples = samples.map_err(|e| e.to_string())?;

    if channels <= 1 {
        return Ok((sample_rate, samples));
    }

    // Downmix
    let frames = samples.len() / channels;
    let mut mono = Vec::with_capacity(frames);
    for f in 0..frames {
        let mut acc = 0i32;
        for c in 0..channels {
            acc += samples[f * channels + c] as i32;
        }
        mono.push((acc / channels as i32) as i16);
    }
    Ok((sample_rate, mono))
}

/// Read impulse response as f32 samples in [-1,1], mono.
pub fn read_impulse_response<P: AsRef<Path>>(path: P) -> Result<(u32, Vec<f32>), String> {
    let (sr, samples) = read_wav_mono_i16(path)?;
    let f32s: Vec<f32> = samples.iter().map(|&s| s as f32 / i16::MAX as f32).collect();
    Ok((sr, f32s))
}

/// Load IR as i16 (for Synthesizer::set_impulse_response which expects i16).
pub fn read_impulse_response_i16<P: AsRef<Path>>(path: P) -> Result<(u32, Vec<i16>), String> {
    read_wav_mono_i16(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::env;

    #[test]
    fn write_and_read_roundtrip() {
        let dir = env::temp_dir();
        let path = dir.join("es_audio_io_test.wav");
        let samples: Vec<i16> = (0..1000).map(|i| (i * 3) as i16).collect();
        write_wav_i16_mono(&path, 44_100, &samples).unwrap();
        let (sr, back) = read_wav_mono_i16(&path).unwrap();
        assert_eq!(sr, 44_100);
        assert_eq!(back.len(), samples.len());
        assert_eq!(back, samples);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn f64_writer_clamps() {
        let dir = env::temp_dir();
        let path = dir.join("es_audio_io_f64.wav");
        write_wav_f64_mono(&path, 48_000, &[0.0, 0.5, -0.5, 2.0, -2.0]).unwrap();
        let (sr, back) = read_wav_mono_i16(&path).unwrap();
        assert_eq!(sr, 48_000);
        assert_eq!(back[0], 0);
        assert_eq!(back[3], i16::MAX);
        assert_eq!(back[4], (-1.0f64 * i16::MAX as f64) as i16);
        let _ = std::fs::remove_file(&path);
    }
}
