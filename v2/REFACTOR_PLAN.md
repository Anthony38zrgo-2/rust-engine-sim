# Refactorización por responsabilidades — `v10-engine-synth` (`v2`)

## 1. Alcance y criterio

* **Origen congelado (`v1`, solo lectura)**: `D:\rust-engine-sim\references\v10-engine-synth\` (respaldado en `references\v10-engine-synth.rar`). Permanece intacto como oráculo de paridad durante toda la migración.
* **Destino (`v2`, construcción limpia)**: `D:\rust-engine-sim\v2\` (paquete aislado con `[workspace]` propio; no altera los crates `crates/es-*` del workspace raíz).
* **Estrategia `side-by-side`**: En lugar de refactorizar *in-place* mezclando módulos nuevos y heredados, cada capa se construye limpia en `v2/` manteniendo API pública, orden de evaluación, coeficientes DSP y sonido actual, validándose contra *golden masters* reales generados desde `v1`.
* **Frontera de alcance**: Durante `H0`–`H5` se preserva el comportamiento V10/GF509 para garantizar paridad exacta con `v1`. Cualquier extensión futura (p. ej. parametrización multi-motor) se apoya sobre `v2` solo después de cerrar `H5`.

---

## 2. Estructura objetivo en `D:\rust-engine-sim\v2`

```text
D:\rust-engine-sim\v2\
├── Cargo.toml                       # [workspace] + package v10-engine-synth (lib v10_engine_synth)
├── REFACTOR_PLAN.md
├── tools/
│   └── v1-baseline/                 # Generador aislado de golden masters contra references/v10-engine-synth
├── examples/
│   ├── exhaust_excitation_probe.rs
│   └── phy061_wave_speed_probe.rs
├── tests/
│   ├── fixtures/                    # Bancos sintéticos deterministas (schema 1 y schema 2 + gearup/geardn)
│   ├── golden/                      # Golden masters PCM/hashes/métricas generados desde v1 en H0
│   ├── parity_v1_v2.rs              # Suite A/B de paridad contra golden masters de v1
│   ├── no_alloc_callback.rs         # Test con CountingAllocator (= 0 allocs en callback tras warmup)
│   └── manifest_validation.rs       # Reglas fail-closed, SHA-256 y rutas relativas seguras
└── src/
    ├── lib.rs                       # Reexports públicos compatibles
    ├── control/
    │   ├── mod.rs
    │   ├── types.rs                 # RuntimeTelemetry, ShiftPhase, TorqueSign, EngineInput
    │   ├── interpolation.rs         # ControlInterpolator (8 continuos suaves + 4 discretos)
    │   ├── energy.rs                # EnergyFollower y target acústico
    │   └── shift_gesture.rs         # ShiftGesture, shift_energy_gain
    ├── config/
    │   ├── mod.rs                   # EngineConfig fachada compatible
    │   ├── physical.rs              # PhysicalEngineConfig + subconfig física
    │   ├── acoustic.rs              # AcousticMixConfig, AcousticSceneConfig
    │   ├── samples.rs               # ThreeZoneSampleLayerConfig
    │   └── runtime.rs               # Gf509RuntimeConfig y EngineMode
    ├── physics/
    │   ├── mod.rs
    │   ├── crank.rs                 # Crankshaft, CYLINDER_COUNT y eventos de encendido
    │   ├── geometry.rs              # SliderCrank
    │   ├── cylinder.rs              # Estado y actualización del cilindro
    │   └── thermodynamics/          # combustion, gas, exhaust_runner, exhaust_valve, polytrope
    ├── synthesis/
    │   ├── mod.rs
    │   ├── procedural.rs            # V10Engine fachada; orquestación por muestra
    │   ├── frame.rs                 # EngineFrame y agrupación interna de señales
    │   ├── turbulence.rs            # Envolvente, RNG determinista y ruido
    │   ├── exhaust.rs               # RunnerWaveguide, Collector
    │   └── structural.rs            # BlockHead y señales mecánicas
    ├── samples/
    │   ├── mod.rs                   # ThreeZoneSampleLayer (API compatible)
    │   ├── bank.rs                  # PreparedSampleBank, zonas, anchors, límites y capacidades precalculadas
    │   ├── playback.rs              # Cursores, phase align, pitch/rate
    │   ├── resampling.rs            # sinc y kernel precomputado
    │   ├── crossfade.rs             # Pesos legacy/multi/variantes/retention en scratch pre-reservado
    │   └── zone_dsp.rs              # ZoneMidProcessor, TrackingNotch
    ├── dsp/
    │   ├── mod.rs
    │   ├── filters.rs               # DcBlocker, OnePoleLowPass, ThreePoleLowPass, Biquad, BandPassNoise
    │   ├── resonators.rs            # Resonator, ModalBank
    │   ├── spectrum.rs              # SpectrumChain, SpectrumChainConfig, UpperMidShelf
    │   └── limiter.rs               # Limitador final de seguridad
    ├── acoustics/
    │   ├── mod.rs
    │   ├── scene.rs                 # AcousticScene y AcousticFrame
    │   ├── cover.rs                 # EngineCover
    │   ├── mounts.rs                # EngineMountMonocoque
    │   └── paths.rs                 # CylinderMechanicalPath y AirPath
    ├── drivetrain/
    │   ├── mod.rs
    │   ├── transmission.rs          # TransmissionSynth (usa control::ShiftPhase)
    │   └── shift_samples.rs         # GearShiftSamples, GearShiftPlayer (usa assets::wav y control::ShiftPhase)
    ├── assets/
    │   ├── mod.rs
    │   ├── manifest.rs              # Parseo schema 1 y 2
    │   ├── validation.rs            # SHA-256 fail-closed y validación de rutas seguras
    │   └── wav.rs                   # read_mono_pcm16, write_mono_pcm16, Pcm16Wav
    ├── runtime/
    │   ├── mod.rs                   # Gf509Runtime fachada pública
    │   ├── renderer.rs              # Pipeline común, buffers y buses
    │   ├── source.rs                # enum EngineSource { Hybrid, SampleOnly }
    │   └── mixing.rs                # blend_hybrid y ganancias
    ├── offline/
    │   ├── mod.rs
    │   ├── scenario.rs              # Curvas de RPM, lift, shift sequence
    │   ├── recorder.rs              # Stems, CSV de telemetría y exportación WAV
    │   └── provenance.rs            # git_head, source_fingerprint
    └── bin/
        ├── v10_render.rs            # CLI delgada -> offline::run(config)
        ├── v10_source_bench.rs
        ├── v10_coast_bench.rs
        └── v10_boundary_regression.rs
```

---

## 3. Hitos de implementación (`H0`–`H5`)

* **H0 (`P0`)**: Baseline + *golden masters* reales ejecutando `v1` (5.000 RPM, barrido RPM, coast/lift, cambios up/down, `Hybrid` y `SampleOnly`, bancos sintéticos schema 1 y 2 + salidas `v10_render`), `v2/Cargo.toml`, esqueleto modular en `v2/src/`, `tests/parity_v1_v2.rs`, `tests/no_alloc_callback.rs`, `tests/manifest_validation.rs`.
* **H1 (`P1`)**: Portar `control/`, `assets/`, `dsp/` y `config/` sin dependencias invertidas ni duplicación de filtros/WAV/validación.
* **H2 (`P2`)**: Portar `samples/` separando `PreparedSampleBank` del reproductor, con `process_multi_into` sin asignaciones en heap + paridad exacta con `v1` + test `no_alloc_callback` = 0 allocs en schema 1 y 2.
* **H3 (`P3`)**: Portar `physics/`, `synthesis/`, `acoustics/` y `drivetrain/` manteniendo orden exacto de las 7 etapas en `V10Engine::render_sample()` y `EngineFrame` compatible con `AcousticScene`.
* **H4 (`P4`)**: Implementar `runtime/` unificado (`Gf509Runtime`, `EngineSource`, `ControlInterpolator`) + paridad `render_block` y 0 allocs.
* **H5 (`P5`)**: Implementar `offline/`, `bin/*`, `examples/*`, validar `cargo fmt --check`, `cargo test --all-targets`, *golden masters* CLI y benchmarks comparativos `v1` vs `v2`.
