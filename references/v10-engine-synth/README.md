# V10 engine synth (greenfield)

Standalone Rust-only V10 acoustic experiment. It intentionally does not depend
on `vehicle-audio-engine`, C++, Faust, Godot, or the existing procedural synth.

The first acceptance target is a fixed 5,000 RPM render built from ten explicit
cylinders, crank-angle combustion pressure, exhaust-valve blowdown, individual
headers, two collectors, and a block/head response.

Run tests:

```powershell
cargo test --manifest-path game/crates/v10-engine-synth/Cargo.toml
```
Render the 5,000 RPM candidate:

```powershell
cargo run --release --manifest-path game/crates/v10-engine-synth/Cargo.toml --bin v10_render -- `
  --rpm 5000 --seconds 6 --warmup 1 --throttle 0.72 --load 0.78 `
  --out reports/audio/rust-greenfield/gf310_5000rpm.wav `
  --stems-dir reports/audio/rust-greenfield/gf310_5000rpm_stems
```
