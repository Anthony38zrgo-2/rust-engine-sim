# Comparación: rust-engine-sim vs references (GF509)

Referencias analizadas: `references/v10-engine-synth` (motor V10 procedural GF509) y
`references/vehicle-audio-engine` (runtime de audio de vehículo que embebe GF509).
Implementación actual: `crates/*` + `engines/rs24_v10_*.json`.

## 0. Resumen

| | rust-engine-sim (actual) | v10-engine-synth (GF509) | vehicle-audio-engine |
| --- | --- | --- | --- |
| Rol | Simulador físico completo + render | Motor V10 procedural standalone | Runtime de mezcla de vehículo (sampler + GF509) |
| Física | Sólidos rígidos 2D + gases, 20 kHz | Cinemática analítica + Wiebe + waveguides, 48 kHz | Curva de par (no termodinámica), tick del juego |
| Audio | 48/96 kHz acústico → 48 kHz | 48 kHz directo (física = audio) | 44.1 kHz sampler + síntesis |
| Cilindros | 10 reales | 10 reales | 5 simulados + 5 derivados |
| Escena | IR por escape + resonancias | 4 rutas estructurales + paths por cilindro | Reverb bus, sin escena 3D |

## 1. Modelo físico

**Actual.** Solver de cuerpos rígidos secuencial-impulsos con proyección de posición
(`crates/es-solver/src/lib.rs`), cámara de combustión con modelo de llama/efficiency
(`es-combustion`), dinámica de gases bidireccional plenum↔runner↔cilindro↔colector
(`es-gas`), 10 cilindros reales, 20 kHz. Audio derivado de la presión del runner de cada
cilindro (`es-sim:1052 stage_exhaust_audio`).

**GF509.** No hay solver de cuerpos: cinemática slider-crank analítica con tabla de
volumen de 1025 entradas (`geometry.rs:14,92-133`, error ≤1e-8 m³). Termodinámica Wiebe
real: `shape 3.0`, `rise 24°`, `decay 112°`, `fuel 600 J`, `efficiency 0.96`,
`ref 17 MPa` (`config.rs:91-104`); `use_physical_pressure=true`. Escape: válvula +
runner lumped (`cylinder.rs:154-183`) + **waveguide por cilindro** (`header_lengths`
0.535–0.578 m, reflexión −0.34, `c = sqrt(γRT)` ≈ 545 m/s, `config.rs:119-125`) + 2
colectores modales (`acoustics.rs:241-375`). Geometría: bore 96 mm, stroke 42 mm, biela
135 mm, CR 12.5, V72° (`config.rs:83-87`).

**vehicle-audio-engine.** `powertrain.rs` es un modelo de curva de par/energía
(`EnergyConfig:134-162`), no termodinámico. La síntesis V10 es half-block: 5 cilindros
simulados + banco derivado con `phase_offset 72°`, `delay 1.5 ms`,
`decorrelation 0.40` (`powertrain.rs:307-336`).

## 2. Generación de audio y política de fase entre bancadas

**Actual.** `pulse = gain · att³ · ((p_runner − ATM) + 0.1·dyn_p) + intake_gain·intake`
(`es-sim:1052`), con ganancias explícitas (`es-sim:729`) y delay por propagación del
primario (`es-sim:270 AudioPathParams`). La simetría física se corrigió (deriva del
mecanismo) para que `rpm/24` cancele; el criterio de validación es `bank ≪ firing`.

**GF509.** Suma `pressure` y `pressure_derivative` por banco, pero aplica **pesos
acústicos asimétricos deliberados** `BANK_A=1.20 / BANK_B=0.80` en derivada y escape
(`engine.rs:431-448`), documentados para excitar el "Order 0.5" (f0/2) a −5.2 dB como en
la grabación onboard del Renault R24. Añade `cylinder_signature` [1.10, 0.94, 1.04, 0.90,
1.07, 0.96, 1.12, 0.92, 1.02, 0.97] y `cycle_variation 0.004` (`config.rs:116-118`),
seguidor de energía (attack 18 ms / release 413 ms) y turbulencia BPF 460–2050 Hz.
Limiter de seguridad soft: knee 0.92, tanh 0.075, cap 0.995 (`engine.rs:478-486`).

**vehicle-audio-engine.** Crossfade de 5 bandas por RPM con pesos triangulares
(`state.rs:157-177`), pitch `0.5–3.5`, anti-alias LP a `0.45·rate`, y para GF509 usa
`continuous_output_gain() = 1.0` (sin la doble atenuación legacy `0.45+0.55·throttle`,
`mixer.rs:368-375`).

## 3. Escena acústica (foco de la comparación)

**Actual.** No hay rutas estructurales: el bus por sistema de escape pasa por IR
(convolución 0.6, HP150) + resonancias biquad opcionales + capas de ruido, y se normaliza.
Hallazgo del sweep wet: una IR distinta por bancada rompe la cancelación de `rpm/24`
(bank +18.5 dB a 18k); con la misma IR en ambos escapes el V10 se mantiene (bank −5.8 dB).

**GF509 (`scene.rs`).** Escena de radiación estructural, mono, sin doppler/oclusión/
reverb/IR:
- 10 `CylinderMechanicalPath` por cilindro: modos base `236 Hz + spread·11.5 + bank·8`,
  delays `0.16 + pos·0.085 + bank·0.055 ms`, **polaridad alterna** para evitar
  cancelación, DC `72+4·pos Hz`, LP `1350+85·pos Hz` (`scene.rs:82-136`).
- `EngineCover`: paneles 684–1982 Hz + skin 2438–5071 Hz, delay 1.309 ms (~0.45 m de
  cabina), LP 6400 Hz (`scene.rs:249`).
- `MountMonocoque`: dos etapas, mount 148–515 Hz (0.18 ms) → monocoque 171–548 Hz (0.82 ms).
- Aire: comb de 3 taps `2.11 / 3.91 / 6.31 ms` con `0.56 / 0.28 / 0.13`, shelf 2500 Hz.
- Split dry 360 Hz LP / 2650 Hz; ganancias `0.18 / 0.46 / 0.14`; salida `2.90`; HP 75 Hz
  final (`scene.rs:47-63,379-462`).
- La frecuencia de encendido se sigue por delta de cigüeñal y alimenta un ramp
  `8500→14500 rpm` que apaga el dry_high (`*0.72`) e inclina el cover (`scene.rs:402-407`).
- Runtime híbrido: escena procedural + `ThreeZoneSampleLayer` (stems tonales/residuales
  crossfadeados por RPM, pitch con sinc ventana radio 4), blend `*0.61`
  (`runtime.rs:19,38-47`).

**vehicle-audio-engine.** Sin escena 3D: `listener_distance` se propaga pero no se aplica
(`mixer.rs:544-548`); solo `pan` equal-power y un bus de reverb compartido (pre-delay 8 ms,
decay 0.55 s, damping 0.58, wet −3 dB). LOD de calidad 25/80/200 m con histéresis
(`powertrain.rs:623-649`).

## 4. Sample rate e interpolación

**Actual.** Sim 20 kHz → render acústico 48/96 kHz con interpolación cúbica de variables
físicas + delay fraccional (`es-dsp:818,848`) → 48 kHz con Butterworth + decimación
(`es-dsp:893`). Las IR de 44.1 kHz se remuestrean al rate acústico (`render.rs:177`).

**GF509.** Un solo rate (48 kHz) para física y audio; resampling solo en la capa de
samples (sinc ventana radio 4, 1024 fases, `sample_layer.rs:7-9`). Interpolación de
telemetría por pasos precomputados (`runtime.rs:358-372`).

**vehicle-audio-engine.** 44.1 kHz, bloques ≤4096 (`mixer.rs:353`), cursor fraccional con
interpolación lineal y wrap de loop (`grand_prix_sampler.rs:1038-1088`).

## 5. Niveles / limitación

| | Actual | GF509 | vehicle-audio-engine |
| --- | --- | --- | --- |
| Control | Leveler opcional (attack/release, sin clamp) + `normalize_peak_dbfs` | Limiter soft fijo + `master_gain 7.0` | Limiter 0.90 + saturation 0.06 |
| Headroom | Normalización a −1 dBFS | Híbrido `*0.61` | `engine_headroom 0.62` |
| Métrica | peak/rms/crest/plateau/clips/flat run | `limiter_reduction_db` | protección + golden |

## 6. Configuración y validación

- **Actual:** JSON por motor con switches de capa (leveler, HF, ruidos, resonancias,
  intake, normalization, rate acústico, delays) y `analyze` de órdenes con alarma 3 dB.
  Tests: mecanismo acotado, primary unificado, tren V10 ideal, cancelación de bancadas,
  delay conocido, resample 96→48, bypass de leveler.
- **GF509:** `EngineConfig` + `Gf509RuntimeConfig` + manifest de sample layer; modo
  `Hybrid`/`SampleOnly`; `v10_render` con stems por capa y JSON de métricas
  (peak/rms/limiter); `v10_boundary_regression`, `v10_coast_bench`, probes de wave speed
  y excitación de escape.
- **vehicle-audio-engine:** `sound_mixer_config.json` schema v2 (ADSR/EQ/tube/reverb
  sends/master/hot reload por sonido), `baseline_golden.json`, `bank_integration`, y
  probes de provenance (mute/bypass/excision de GF509, `v10_provenance_probe.rs`).

## 7. Brechas y oportunidades

1. **Escena estructural:** adoptar rutas cover/mount/monocoque y paths por cilindro con
   polaridad alterna (GF509) sobre nuestro bus por escape; hoy dependemos de la IR.
2. **Asimetría de bancada como parámetro perceptual:** GF509 fija Order 0.5 a −5.2 dB;
   nuestro wet quedó en −5.8 dB. Exponer un `bank_asymmetry` configurable en vez de solo
   cancelar.
3. **Capa híbrida de samples** crossfadeada por RPM (GF509/GP) para textura, ausente aquí.
4. **Contratos de provenance y golden baseline** (mute/bypass/excision, hash por banco)
   para regresión de audio.
5. **Waveguide 1D por primario** con `c=sqrt(γRT)` y reflexión −0.34: nuestro runner es
   un volumen lumped; la fase del primario es solo delay.
6. **Ventajas actuales:** solver físico real con proyección, gas bidireccional, IR/escena
   configurables, rate acústico separado con delay fraccional, análisis de órdenes y
   normalización/leveler explícitos.
