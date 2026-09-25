# RS24 V10 audio fix — reporte de ejecución

## 1. Archivos modificados

| Archivo | Cambio |
| --- | --- |
| `crates/es-solver/src/lib.rs` | Proyección de posición (NGS) en `Constraint::project` + `PROJECTION_ITERATIONS`; prioridad de solver para fuentes de velocidad; test de vínculo corregido |
| `crates/es-sim/src/lib.rs` | `AudioPathParams` (delay de header/output/listener, `equal_bank_delay`); registro por cilindro de `base`, `dyn_p`, `att3`, `intake`; ganancias acústicas explícitas; accessors de primary/delay; tests |
| `crates/es-intake-exhaust/src/lib.rs` | `ExhaustParams.header_loss_gain`, `collector_loss_gain`, `exhaust_output_gain` |
| `crates/es-dsp/src/lib.rs` | `render_pulse_channels` (interpolación cúbica + delay fraccional), `render_offline_to` (rate acústico + decimación), leveler con `enabled/attack/release` sin clamp duro, rama HF normalizada, capas `flow_noise`/`mechanical_noise`, `Biquad` de resonancias, normalización final; tests |
| `crates/es-config/src/lib.rs` | Primary unificado (`exhaust.primary_tube_length_mm`, fallback a banco), `AudioFile` con rate acústico, leveler, HF, ruidos, resonancias, `intake_gain`, normalización; tests |
| `crates/es-config/src/bin/render.rs` | Pipeline acústico, `--acoustic-rate`, `--stems`, envolvente de flujo, resonancias, métricas (rms/crest/plateau/flat run) |
| `crates/es-config/src/bin/analyze.rs` | Análisis de órdenes por ventana (crank/bank/firing/2..7x), centroid, rolloff, crest, clips, plateau, alarma 3 dB, JSON/Markdown |
| `engines/rs24_v10_acoustic_reference.json` | Nuevo baseline seco (48/96k, sin ruido/IR/leveler, delays y ganancias iguales) |
| `engines/rs24_v10_colored.json` | Variante con capas tonales opcionales (HF, ruidos, intake, resonancias) |
| `engines/rs24_v10_sweep_wet.json` | Sweep wet: capas tonales + IR HP150 (convolution 0.6), misma IR en ambas bancadas |

## 2. Causas raíz corregidas

1. **Deriva del mecanismo** (`es-solver`): los impulsos a nivel de velocidad dejaban error geométrico que crecía con el régimen (pistones a ~0.95 m con biela de 0.09 m a 18k rpm). Una bancada quedaba sin compresión y la otra disparaba, generando el dominio de `rpm/24`. Se añadió proyección de posición tras integrar; el mecanismo queda simétrico (combustión idéntica por bancada).
2. **Fase entre bancadas** (`es-sim`): el delay usaba `header_primary + exhaust.length()`, con 0.63 ms de diferencia entre bancadas. Ahora la fase usa solo propagación del primario; el largo del escape no es un delay puro (`include_output_path_delay=false`), con opción de diagnóstico `equal_bank_delay`.
3. **`1/length²` en el pulso** (`es-sim`): eliminado; sustituido por `header_loss_gain * collector_loss_gain * exhaust_output_gain` (default 1.0).
4. **Primary duplicado** (`es-config`): `bank.primary_length_mm` (40/45 mm) y `exhaust.primary_tube_length_mm` (508 mm) alimentaban audio y física por separado. Ahora 508 mm es la única fuente para cámara, delay y `DelayLine` (con override per-cilindro y fallback al banco).
5. **Resolución temporal** (`es-dsp`/`render`): sim 20 kHz → render acústico 48/96 kHz con interpolación cúbica de las variables físicas y delay fraccional, decimación band-limited a 48 kHz. No es upsampling del WAV.
6. **Leveler**: sin clamp duro; `enabled/attack/release`; normalización final en f32. El render de referencia no tiene mesetas ni clips.
7. **Rama HF**: derivada normalizada por `hf_reference_hz`; `hf_mix` estable en 0..1.
8. **Ruido**: capas independientes `flow_noise` (con envolvente física) y `mechanical_noise`, ya no modulación multiplicativa de la señal.

## 3. Tests añadidos

- `es-sim`: `mechanism_stays_bounded_at_speed`, `mechanism_stays_bounded_at_high_rpm`.
- `es-config`: `exhaust_primary_feeds_gas_and_audio_identically`, `bank_primary_is_fallback_when_exhaust_length_absent`.
- `es-dsp`: `v10_ideal_train_dominated_by_firing_order`, `banks_offset_half_period_cancel_bank_order`, `known_delay_changes_bank_versus_firing_ratio`, `resample_96_to_48_keeps_harmonics_without_aliases`, `disabled_leveler_leaves_no_digital_plateaus`, `leveler_bypasses_when_disabled`, `leveler_handles_pathological_input`, `hf_branch_is_frequency_normalized`.
- `es-solver`: `link_keeps_distance` corregido (puntos con offset, distancia fija real).

## 4. cargo fmt / check / test

- `cargo fmt` (crates modificados): OK.
- `cargo check --workspace`: OK, sin warnings.
- `cargo test --workspace`: 24 binarios de test, todos `ok`.

## 5. Renders

| Render | Comando | Resultado |
| --- | --- | --- |
| Baseline original | código previo, `rs24_v10_sweep_no_ir.json` | `output/analysis_rs24/baseline_no_ir.wav` (SHA256 `2144ABF6…`) |
| Corregido (referencia) | `render engines/rs24_v10_acoustic_reference.json` | `output/rs24_v10_acoustic_reference.wav` |
| Corregido (legacy no-IR) | `render engines/rs24_v10_sweep_no_ir.json` | `output/analysis_rs24/corrected_no_ir.wav` |
| Con color tonal | `render engines/rs24_v10_colored.json` | `output/rs24_v10_colored.wav` |
| Sweep wet | `render engines/rs24_v10_sweep_wet.json` | `output/rs24_v10_sweep_wet.wav` |
| Stems | `render … --stems output/analysis_rs24/stems` | `exhaust_0.wav`, `exhaust_1.wav`, `mix.wav` |

La IR se remuestrea de 44.1 kHz al rate acústico antes de convolucionar. Una IR distinta por bancada rompe la cancelación de `rpm/24` (bank +18.5 dB a 18k); con la misma IR en ambos escapes el orden de encendido se mantiene dominante (bank −5.8 dB).

## 6. Comparación espectral (relativo a firing, dB)

| RPM | Orden | Baseline | Corregido (referencia) |
| --- | --- | --- | --- |
| 10000 | bank | −1.86 | −25.84 |
| 10000 | 2xf | −10.54 | −22.31 |
| 10000 | 3xf | −14.30 | −19.09 |
| 10000 | 4xf | −17.46 | −32.83 |
| 14000 | bank | **+7.50** | −13.31 |
| 14000 | 2xf | +0.96 | −5.96 |
| 14000 | 3xf | −7.03 | −22.96 |
| 14000 | 4xf | −4.67 | −21.58 |
| 18000 | bank | **+10.53** | −15.41 |
| 18000 | 2xf | −3.37 | −6.00 |
| 18000 | 3xf | −12.37 | −14.82 |
| 18000 | 4xf | −20.84 | −26.77 |
| 18000 | 5xf | −23.56 | −28.85 |
| 18000 | 6xf | −29.31 | −34.67 |

Ventanas con alarma `bank − firing > 3 dB`: **86 → 9**.

## 7. Métricas de clipping / leveler

| Render | peak | plateau | clips | flat run máx | crest |
| --- | --- | --- | --- | --- | --- |
| Baseline | 30303 | 2.66 % | 0 | — | — |
| Referencia (leveler off, norm −1 dBFS) | 29204 | 0.00 % | 0 | 1133 | 6.10 |
| Legacy no-IR (leveler on) | 32768 | 1.15 % | 2185 | 295 | 2.45 |

El legacy conserva su EQ/leveler original; la referencia seca es el baseline válido para cambios futuros.

## 8. Rate acústico (validación P0-D)

A 18 000 rpm el orden 2xf queda en −5.99 dB (48 kHz) y −6.22 dB (20 kHz); 6xf −34.7 dB (48 kHz) vs −30.6 dB (20 kHz). No aparecen componentes plegadas por debajo de Nyquist; el contenido > 10 kHz sólo existe en los renders de 48/96 kHz.

## 9. Integración Fases 1+2 (escena estructural GF509 + waveguide)

**Archivos nuevos**
- `crates/es-dsp/src/modal.rs`: `Resonator`, `ModalBank`, `DcBlocker`, `OnePoleLowPass` (port de `acoustics.rs`).
- `crates/es-dsp/src/scene.rs`: `StructuralScene` con rutas aire (comb 2.11/3.91/6.31 ms), cover (684–5071 Hz, 1.309 ms), mount/monocoque (148–548 Hz, 0.18/0.82 ms) y 10 paths por cilindro con polaridad alterna; split 360/2650 Hz, gains GF509.
- `crates/es-dsp/src/waveguide.rs`: `RunnerWaveguide` (delay + reflexión −0.34 + `c=sqrt(γRT)` + loss 0.32).

**Cambios**
- `es-sim`: `CylinderAudioSeries` graba `derivative`, `blowdown`, `flow`, `runner_temp`; `SimOutput` expone `cylinder_bank`, `cylinder_primary_length` y `collector_pressure`.
- `es-dsp`: `render_scene` con interpolación cúbica, auto-calibración (`excitation_scale 0.1`, normalización por máximos, `scene_scale`), y `Biquad::high_shelf/highpass`.
- `es-config`/`render.rs`: `audio.scene`, `audio.bank_gain`, `audio.header_waveguide`; la escena entra como canal extra y exporta stems `scene_air/cover/mount/cylinder/header/mix`.
- Config de ejemplo: `engines/rs24_v10_scene.json` (96 kHz acústico, normalizado −1 dBFS, sin leveler).

**Resultado (18k rpm, relativo a firing)**

| Orden | Referencia seca | Escena+waveguide |
| --- | --- | --- |
| bank | −15.4 dB | **−5.12 dB** (objetivo GF509 −5.2) |
| 2xf | −6.0 | −4.5 |
| 3xf | −14.8 | −13.1 |
| crank | −16.5 | −3.4 |

10k: bank −26.9; 14k: bank −25.5. Firing dominante en todo el barrido; 13 ventanas en alarma (vs 9 de la referencia). Métricas: peak 29204 (−1 dBFS), rms 7203, crest 4.05, 0 clips, 0 mesetas.

**Calibración**: `bank_gain [1.09, 0.91]`, `excitation_gain 1.0` (la normalización del flujo cambia la escala respecto a GF509), `header_gain 0.5`, `engine_air_gain 0.5`, `cover 0.30`, `mount 0.04`.

## 10. Pendiente

- Commits atómicos por grupo funcional: no creados (requiere confirmación explícita).
- IR/convolution: configurable por escape (`impulse_response`), desactivado en la referencia por diseño (se reintroduce al final).
