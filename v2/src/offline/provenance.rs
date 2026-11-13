//! Git revision and source fingerprint helpers for offline render metadata.

use std::fs;
use std::path::Path;
use std::process::Command;

use sha2::{Digest, Sha256};

const V2_SOURCE_PATHS: &[&str] = &[
    "Cargo.toml",
    "src/lib.rs",
    "src/acoustics/mod.rs",
    "src/acoustics/cover.rs",
    "src/acoustics/mounts.rs",
    "src/acoustics/paths.rs",
    "src/acoustics/scene.rs",
    "src/assets/mod.rs",
    "src/assets/manifest.rs",
    "src/assets/validation.rs",
    "src/assets/wav.rs",
    "src/config/mod.rs",
    "src/config/acoustic.rs",
    "src/config/physical.rs",
    "src/config/runtime.rs",
    "src/config/samples.rs",
    "src/control/mod.rs",
    "src/control/energy.rs",
    "src/control/interpolation.rs",
    "src/control/shift_gesture.rs",
    "src/control/types.rs",
    "src/drivetrain/mod.rs",
    "src/drivetrain/shift_samples.rs",
    "src/drivetrain/transmission.rs",
    "src/dsp/mod.rs",
    "src/dsp/filters.rs",
    "src/dsp/limiter.rs",
    "src/dsp/resonators.rs",
    "src/dsp/spectrum.rs",
    "src/offline/mod.rs",
    "src/offline/provenance.rs",
    "src/offline/recorder.rs",
    "src/offline/scenario.rs",
    "src/physics/mod.rs",
    "src/physics/crank.rs",
    "src/physics/cylinder.rs",
    "src/physics/geometry.rs",
    "src/physics/thermodynamics/mod.rs",
    "src/physics/thermodynamics/combustion.rs",
    "src/physics/thermodynamics/exhaust_runner.rs",
    "src/physics/thermodynamics/exhaust_valve.rs",
    "src/physics/thermodynamics/gas.rs",
    "src/physics/thermodynamics/polytrope.rs",
    "src/runtime/mod.rs",
    "src/runtime/mixing.rs",
    "src/runtime/renderer.rs",
    "src/runtime/source.rs",
    "src/samples/mod.rs",
    "src/samples/bank.rs",
    "src/samples/crossfade.rs",
    "src/samples/playback.rs",
    "src/samples/resampling.rs",
    "src/samples/zone_dsp.rs",
    "src/synthesis/mod.rs",
    "src/synthesis/exhaust.rs",
    "src/synthesis/frame.rs",
    "src/synthesis/procedural.rs",
    "src/synthesis/structural.rs",
    "src/synthesis/turbulence.rs",
    "src/bin/v10_render.rs",
];

/// Returns current git HEAD SHA, or `"unknown"` when git is unavailable.
pub fn git_head() -> String {
    Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .map(|out| String::from_utf8_lossy(&out.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".into())
}

/// Returns `true` when the working tree has uncommitted git changes.
pub fn source_dirty() -> bool {
    Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .map(|out| !String::from_utf8_lossy(&out.stdout).trim().is_empty())
        .unwrap_or(true)
}

/// Computes a deterministic SHA-256 fingerprint across v2 source files, or
/// `"unavailable"` when none of the candidate source roots exist from the working directory.
pub fn source_fingerprint() -> String {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let roots = [Path::new("."), Path::new("v2"), manifest_dir];
    let Some(root) = roots
        .into_iter()
        .find(|base| base.join("src/lib.rs").is_file())
    else {
        return "unavailable".into();
    };
    let mut hasher = Sha256::new();
    let mut hashed_any = false;
    for rel in V2_SOURCE_PATHS {
        let Ok(bytes) = fs::read(root.join(rel)) else {
            continue;
        };
        hashed_any = true;
        hasher.update(rel.as_bytes());
        hasher.update(b":");
        hasher.update(&bytes);
    }
    if !hashed_any {
        return "unavailable".into();
    }
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}
