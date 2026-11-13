use std::path::Path;

use crate::runtime::ShiftPhase;
use crate::sample_layer::read_mono_pcm16;

pub struct GearShiftSamples {
    pub(crate) up: Vec<f32>,
    pub(crate) down: Vec<f32>,
}

impl GearShiftSamples {
    pub fn load_directory(directory: &Path, expected_sample_rate: u32) -> Result<Option<Self>, String> {
        let up_path = directory.join("gearup.wav");
        let down_path = directory.join("geardn.wav");
        if !up_path.is_file() && !down_path.is_file() {
            return Ok(None);
        }
        let up = if up_path.is_file() {
            Some(read_mono_pcm16(&up_path)?)
        } else {
            None
        };
        let down = if down_path.is_file() {
            Some(read_mono_pcm16(&down_path)?)
        } else {
            None
        };
        for (label, wav) in [("gearup", up.as_ref()), ("geardn", down.as_ref())] {
            if let Some(wav) = wav {
                if wav.sample_rate != expected_sample_rate {
                    return Err(format!(
                        "{label} sample rate {} does not match runtime rate {expected_sample_rate}",
                        wav.sample_rate
                    ));
                }
                if wav.samples.len() < 128 {
                    return Err(format!("{label} is too short"));
                }
            }
        }
        Ok(Some(Self {
            up: up.map(|wav| wav.samples).unwrap_or_default(),
            down: down.map(|wav| wav.samples).unwrap_or_default(),
        }))
    }
}

pub struct GearShiftPlayer {
    samples: Option<GearShiftSamples>,
    gain: f32,
    reference_rpm: f32,
    rate: f32,
    rate_alpha: f32,
    up_active: bool,
    up_cursor: f32,
    down_active: bool,
    down_cursor: f32,
    last_phase: u8,
}

const FADE_IN_SAMPLES: f32 = 88.0;
const FADE_OUT_SAMPLES: f32 = 352.0;
const MIN_RATE: f32 = 0.5;
const MAX_RATE: f32 = 2.0;
const RATE_SMOOTH_SECONDS: f32 = 0.02;

#[inline]
fn fade_envelope(cursor: f32, length: usize) -> f32 {
    let attack = if cursor < FADE_IN_SAMPLES {
        cursor / FADE_IN_SAMPLES
    } else {
        1.0
    };
    let release = if length as f32 - cursor <= FADE_OUT_SAMPLES {
        (length as f32 - cursor) / FADE_OUT_SAMPLES
    } else {
        1.0
    };
    attack * release
}

#[inline]
fn read_interpolated(samples: &[f32], cursor: f32) -> f32 {
    let index = cursor as usize;
    if index + 1 >= samples.len() {
        return samples[samples.len() - 1];
    }
    let fraction = cursor - index as f32;
    samples[index] * (1.0 - fraction) + samples[index + 1] * fraction
}

impl GearShiftPlayer {
    pub fn new(
        samples: Option<GearShiftSamples>,
        gain: f32,
        reference_rpm: f32,
        sample_rate: u32,
    ) -> Result<Self, String> {
        if !gain.is_finite() || !(0.0..=2.0).contains(&gain) {
            return Err(format!("gear shift gain out of range: {gain}"));
        }
        if !reference_rpm.is_finite() || reference_rpm <= 0.0 {
            return Err(format!("gear shift reference rpm out of range: {reference_rpm}"));
        }
        let rate_alpha = 1.0 - (-1.0 / (RATE_SMOOTH_SECONDS * sample_rate as f32)).exp();
        Ok(Self {
            samples,
            gain,
            reference_rpm,
            rate: 1.0,
            rate_alpha,
            up_active: false,
            up_cursor: 0.0,
            down_active: false,
            down_cursor: 0.0,
            last_phase: 0,
        })
    }

    pub fn reset(&mut self) {
        self.rate = 1.0;
        self.up_active = false;
        self.up_cursor = 0.0;
        self.down_active = false;
        self.down_cursor = 0.0;
        self.last_phase = 0;
    }

    #[inline]
    pub fn process(&mut self, phase: ShiftPhase, rpm: f32) -> f32 {
        let phase = phase as u8;
        if let Some(samples) = &self.samples {
            if phase == 1 && self.last_phase != 1 && !samples.up.is_empty() {
                self.up_active = true;
                self.up_cursor = 0.0;
            }
            if phase == 3 && self.last_phase != 3 && !samples.down.is_empty() {
                self.down_active = true;
                self.down_cursor = 0.0;
            }
            let target = if rpm > 0.0 {
                (rpm / self.reference_rpm).clamp(MIN_RATE, MAX_RATE)
            } else {
                MIN_RATE
            };
            self.rate += self.rate_alpha * (target - self.rate);
        }
        self.last_phase = phase;
        let mut output = 0.0;
        if let Some(samples) = &self.samples {
            if self.up_active {
                output += read_interpolated(&samples.up, self.up_cursor)
                    * fade_envelope(self.up_cursor, samples.up.len())
                    * self.gain;
                self.up_cursor += self.rate;
                if self.up_cursor >= samples.up.len() as f32 {
                    self.up_active = false;
                }
            }
            if self.down_active {
                output += read_interpolated(&samples.down, self.down_cursor)
                    * fade_envelope(self.down_cursor, samples.down.len())
                    * self.gain;
                self.down_cursor += self.rate;
                if self.down_cursor >= samples.down.len() as f32 {
                    self.down_active = false;
                }
            }
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wav::write_mono_pcm16;
    use std::path::PathBuf;

    const SAMPLE_RATE: u32 = 44_100;

    fn test_directory(label: &str) -> PathBuf {
        let mut directory = std::env::temp_dir();
        directory.push(format!(
            "v10_gear_shift_test_{}_{}",
            std::process::id(),
            label
        ));
        std::fs::create_dir_all(&directory).unwrap();
        directory
    }

    fn write_test_samples(directory: &Path) {
        let up: Vec<f32> = (0..2_000)
            .map(|index| (index as f32 / 2_000.0 * 0.8).sin())
            .collect();
        let down: Vec<f32> = (0..1_000)
            .map(|index| ((index as f32 / 1_000.0) * 0.5).cos())
            .collect();
        write_mono_pcm16(&directory.join("gearup.wav"), SAMPLE_RATE, &up).unwrap();
        write_mono_pcm16(&directory.join("geardn.wav"), SAMPLE_RATE, &down).unwrap();
    }

    #[test]
    fn missing_files_load_as_none() {
        let directory = test_directory("empty").join("empty");
        std::fs::create_dir_all(&directory).unwrap();
        assert!(
            GearShiftSamples::load_directory(&directory, SAMPLE_RATE)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn loads_and_plays_one_shot_per_edge() {
        let directory = test_directory("plays");
        write_test_samples(&directory);
        let samples = GearShiftSamples::load_directory(&directory, SAMPLE_RATE)
            .unwrap()
            .unwrap();
        assert_eq!(samples.up.len(), 2_000);
        assert_eq!(samples.down.len(), 1_000);
        let mut player = GearShiftPlayer::new(Some(samples), 0.5, 10_000.0, SAMPLE_RATE).unwrap();
        let mut up_seen: f32 = 0.0;
        for _ in 0..2_500 {
            up_seen = up_seen.max(player.process(ShiftPhase::UpshiftCut, 10_000.0).abs());
        }
        assert!(up_seen > 0.0, "upshift sample must play");
        let mut down_seen: f32 = 0.0;
        for _ in 0..1_500 {
            down_seen = down_seen.max(player.process(ShiftPhase::DownshiftCut, 10_000.0).abs());
        }
        assert!(down_seen > 0.0, "downshift sample must play");
    }

    #[test]
    fn playback_rate_tracks_rpm() {
        let directory = test_directory("rate");
        write_test_samples(&directory);
        let samples = GearShiftSamples::load_directory(&directory, SAMPLE_RATE)
            .unwrap()
            .unwrap();
        let mut player = GearShiftPlayer::new(Some(samples), 1.0, 10_000.0, SAMPLE_RATE).unwrap();
        let duration = |player: &mut GearShiftPlayer, rpm: f32| {
            player.process(ShiftPhase::None, rpm);
            player.process(ShiftPhase::UpshiftCut, rpm);
            let mut frames = 0usize;
            while player.up_active {
                player.process(ShiftPhase::UpshiftCut, rpm);
                frames += 1;
            }
            frames
        };
        let native = duration(&mut player, 10_000.0);
        assert!((native as f32 - 2_000.0).abs() < 5.0, "native duration {native}");
        let slowed = duration(&mut player, 5_000.0);
        assert!(
            slowed > 2_500 && slowed <= 4_000,
            "half-rate duration must stretch the blip, got {slowed}"
        );
        let speeded = duration(&mut player, 20_000.0);
        assert!(
            (1_000..=2_000).contains(&speeded),
            "2x clamp must compress the blip, got {speeded}"
        );
    }

    #[test]
    fn edge_fires_once_and_retriggers_on_new_shift() {
        let directory = test_directory("edge");
        write_test_samples(&directory);
        let samples = GearShiftSamples::load_directory(&directory, SAMPLE_RATE)
            .unwrap()
            .unwrap();
        let mut player = GearShiftPlayer::new(Some(samples), 1.0, 10_000.0, SAMPLE_RATE).unwrap();
        player.process(ShiftPhase::UpshiftCut, 10_000.0);
        player.process(ShiftPhase::UpshiftCut, 10_000.0);
        assert!((player.up_cursor - 2.0).abs() < 0.01, "no retrigger while still in cut");
        player.process(ShiftPhase::None, 10_000.0);
        player.process(ShiftPhase::UpshiftCut, 10_000.0);
        assert!((player.up_cursor - 1.0).abs() < 0.01, "retrigger after leaving cut");
    }

    #[test]
    fn zero_gain_is_silent_and_none_samples_are_silent() {
        let directory = test_directory("silent");
        write_test_samples(&directory);
        let samples = GearShiftSamples::load_directory(&directory, SAMPLE_RATE)
            .unwrap()
            .unwrap();
        let mut muted = GearShiftPlayer::new(Some(samples), 0.0, 10_000.0, SAMPLE_RATE).unwrap();
        for _ in 0..500 {
            assert_eq!(muted.process(ShiftPhase::UpshiftCut, 10_000.0), 0.0);
        }
        let mut empty = GearShiftPlayer::new(None, 1.0, 10_000.0, SAMPLE_RATE).unwrap();
        for _ in 0..500 {
            assert_eq!(empty.process(ShiftPhase::DownshiftCut, 10_000.0), 0.0);
        }
    }

    #[test]
    fn rejects_out_of_range_gain_and_rate_mismatch() {
        assert!(GearShiftPlayer::new(None, 3.0, 10_000.0, SAMPLE_RATE).is_err());
        assert!(GearShiftPlayer::new(None, 1.0, 0.0, SAMPLE_RATE).is_err());
        let directory = test_directory("rejects");
        write_test_samples(&directory);
        assert!(
            GearShiftSamples::load_directory(&directory, 48_000).is_err(),
            "rate mismatch must error"
        );
    }
}