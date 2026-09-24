# RS24 V10: render sin retumbo de las IR

Resultado final: `output/rs24_v10_sweep_bass_tamed.wav` (10 s, 44.1 kHz, mono PCM 16). Configuración: `engines/rs24_v10_sweep_bass_tamed.json`.

Se filtraron ambas respuestas de impulso con un pasa altos Butterworth causal de 4.º orden a 150 Hz; los originales siguen intactos. `smooth_02` requirió escala 0.5898 después del filtrado para evitar saturar el formato PCM 16 de la IR. El DSP sigue usando los primeros 10000 taps de cada una. Se ajustó la mezcla IR/seca de 60/40 a 50/50 y el volumen de salida a 1.2 para comparar al mismo RMS con el render anterior.

Evidencia: entre 3 y 4 s la energía bajo 200 Hz cayó de 55.98% a 6.79% (véase `bass_tamed_metrics.json` para el valor preciso). La diferencia de nivel RMS global es de aproximadamente -0.05 dB. El nuevo WAV no satura PCM y alcanza 18000 RPM. La respuesta de impulso original y el render anterior siguen disponibles para comparación A/B.

No hay herramienta de escucha perceptual disponible en este entorno. Los WAV están preparados al mismo nivel; la aprobación del timbre exige escuchar los dos archivos. La medición RMS global no garantiza igualdad de sonoridad percibida en cada instante porque cambió mucho el espectro y la envolvente.
