# Origen del grave en el render RS24 corregido

La separaciÃ³n de la misma configuraciÃ³n con `convolution=0` (seca) y `convolution=1` (sÃ³lo IR) permite localizar el problema. Se usÃ³ el ejecutable release existente, sin cambios al cÃ³digo fuente ni a los WAV originales.

En la ventana 3â€“4 s, cerca de 8600 RPM, el contenido bajo 200 Hz es 3.8% en seco, 83.6% con IR y 56.0% en la mezcla 60% IR / 40% seca. La frecuencia mÃ¡s fuerte de la mezcla ronda 70 Hz; la cadencia por bancada esperada es aproximadamente 359 Hz y la total del V10 es aproximadamente 718 Hz. La señal seca tiene un pico cerca de 361 Hz, coherente con la cadencia por bancada. La forma y el balance del grave vienen principalmente de la convoluciÃ³n.

Las respuestas smooth_01.wav y smooth_02.wav duran 1.415 s y 1.032 s, pero el DSP usa solo los primeros 10000 taps (0.227 s). Dichos tramos concentran respectivamente 98.8% y 79.6% de su energÃ­a bajo 200 Hz. A pesar de `impulse_response_volume=0.01`, la suma de 10000 taps da ganancias muy elevadas en resonancias graves: smooth_01 tiene mÃ³dulo ~22 a 20 Hz frente a ~0.105 a 750 Hz; smooth_02 ~24 frente a ~1.19. Esta cifra es la magnitud de su respuesta FIR y no una ganancia global uniforme.

El leveler del DSP sigue el pico con una caÃ­da de 0.999 por muestra y actualiza la ganancia hacia arriba con peso 0.1 por muestra. La IR grave dicta los picos y por tanto la ganancia del resto de Ã³rdenes. Puede contribuir a una sensaciÃ³n de grave comprimido o bombeante. Se trata de una inferencia a partir del cÃ³digo y la seÃ±al; no se aislÃ³ el leveler en una variante propia.

El WAV final no satura el formato PCM (pico 23850 en i16). La distorsiÃ³n reportada no requiere recorte a Â±32767: el fuerte color de las IR y la ganancia variable bastan como explicaciÃ³n plausible. No se ha validado con una escucha humana.

CorrecciÃ³n sugerida para la siguiente iteraciÃ³n: seleccionar IR con respuesta neutra o aplicar filtrado pasa altos a cada IR antes de la convoluciÃ³n, y luego volver a ajustar mezcla hÃºmeda/seca y ganancia del leveler a nivel equivalente. Mantener la seÃ±al seca como referencia y evaluar perceptualmente el resultado.
