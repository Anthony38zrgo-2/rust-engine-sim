//! Engine/sample mixing helpers and headroom scaling for GF509 runtime buses.

pub use crate::assets::GF509_HEADROOM_GAIN;

/// Blends the procedural acoustic scene with the optional sample layer.
///
/// The procedural side already carries the synthesized shift gesture inside
/// `V10Engine`; the sample contribution is multiplied by the same `gesture_gain`
/// so hybrid mode wobbles uniformly on recovery/blip. Without a sample layer,
/// `scene` is returned untouched.
#[inline]
pub fn blend_hybrid(
    scene: f32,
    sample: Option<f32>,
    physical_weight: f32,
    sample_weight: f32,
    gesture_gain: f32,
) -> f32 {
    match sample {
        Some(sample) => {
            (scene * physical_weight + sample * sample_weight * gesture_gain) * GF509_HEADROOM_GAIN
        }
        None => scene,
    }
}
