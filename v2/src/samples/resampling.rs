//! Windowed-sinc resampling (`sinc_sample` reference and precomputed radius-4
//! lookup table `sinc_sample_tabled`).

use std::f64::consts::PI;

pub const SINC_RADIUS: isize = 4;
pub const SINC_TABLE_PHASES: usize = 1024;

/// Evaluates the Hann-windowed sinc weight for distance `distance` and `radius`.
#[inline]
pub fn windowed_sinc_weight(distance: f64, radius: isize) -> f64 {
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

/// Evaluates the normalized Hann-windowed sinc kernel directly at fractional `cursor`.
#[inline]
pub fn sinc_sample(samples: &[f32], cursor: f64) -> f32 {
    let base = cursor.floor() as isize;
    let fraction = cursor - cursor.floor();
    let mut output = 0.0f64;
    let mut weight_sum = 0.0f64;
    for tap in -SINC_RADIUS + 1..=SINC_RADIUS {
        let distance = tap as f64 - fraction;
        let weight = windowed_sinc_weight(distance, SINC_RADIUS);
        let index = (base + tap).rem_euclid(samples.len() as isize) as usize;
        output += samples[index] as f64 * weight;
        weight_sum += weight;
    }
    (output / weight_sum.max(1.0e-12)) as f32
}

/// Precomputes one normalized kernel table row per fractional phase (`phases + 1` rows).
/// Built once on the control thread at bank load time.
pub fn build_sinc_table(phases: usize, radius: isize) -> Vec<f32> {
    let taps = (2 * radius) as usize;
    let tap_lo = -radius + 1;
    let mut table = vec![0.0f32; (phases + 1) * taps];
    for phase in 0..=phases {
        let fraction = phase as f64 / phases as f64;
        let mut sum = 0.0f64;
        for tap in 0..taps {
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

/// Evaluates windowed-sinc interpolation by linearly blending adjacent rows of `table`.
#[inline]
pub fn sinc_sample_tabled(
    samples: &[f32],
    cursor: f64,
    table: &[f32],
    phases: usize,
    tap_lo: isize,
    taps: usize,
) -> f32 {
    let base = cursor.floor() as isize;
    let position = (cursor - cursor.floor()) * phases as f64;
    let phase0 = (position.floor() as usize).min(phases - 1);
    let phase1 = phase0 + 1;
    let blend = (position - position.floor()) as f32;
    let row0 = phase0 * taps;
    let row1 = phase1 * taps;
    let mut output = 0.0f32;
    for tap in 0..taps {
        let weight = table[row0 + tap] + (table[row1 + tap] - table[row0 + tap]) * blend;
        let index = (base + tap_lo + tap as isize).rem_euclid(samples.len() as isize) as usize;
        output += samples[index] * weight;
    }
    output
}
