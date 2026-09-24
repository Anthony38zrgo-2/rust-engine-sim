# Rust engine simulator: RS24 V10 audio

This workspace simulates an engine at 20 kHz and renders mono PCM audio at
44.1 kHz. The renderer is `crates/es-config/src/bin/render.rs`; the audio
chain is in `crates/es-dsp/src/lib.rs`.

## Reproduce the current sweep

```sh
cargo run --release -p es-config --bin render -- engines/rs24_v10_sweep_no_ir.json
```

The command writes `output/rs24_v10_sweep_no_ir.wav`. To inspect a WAV:

```sh
cargo run -p es-config --bin analyze -- output/rs24_v10_sweep_no_ir.wav
```

The four 10-second sweep variants are paired with configurations of the same
stem in `engines/`:

| Audio | Purpose |
| --- | --- |
| `output/rs24_v10_sweep.wav` | Initial 5,000–18,000 RPM sweep |
| `output/rs24_v10_sweep_revised.wav` | Jitter removed, input filter raised, RPM steps reduced to 10 ms |
| `output/rs24_v10_sweep_bass_tamed.wav` | 150 Hz high-pass filtered impulse responses and 50% wet mix |
| `output/rs24_v10_sweep_no_ir.wav` | No impulse responses or convolution; direct exhaust signal |

The last two variants match in full-file RMS within 0.001 dB. The user still
reports an audible problem in the bass-tamed variant; the no-IR variant has
not received perceptual approval. It is a diagnostic reference, not a final
sound. No PCM clipping was observed in these two WAVs.

The original impulse responses are in `assets/sound-library/smooth/` and the
filtered variants are in `assets/sound-library/rs24/`. The originals were not
modified. Dry-only and wet-only isolation WAVs are in
`output/analysis_rs24/` along with their configurations. Numerical analysis
and comparisons are in
`output/analysis_rs24/bass_analysis.md`,
`output/analysis_rs24/bass_tamed_readme.md`, and
`output/analysis_rs24/no_ir_metrics.json`.

`cargo check --workspace` passes on the current source. Rendered audio still
requires human listening before it is accepted as a faithful RS24 sound.
