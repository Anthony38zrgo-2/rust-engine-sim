from pathlib import Path
import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('output/analysis_rs24')
files=[('Seca (sin IR)',root/'rs24_dry_isolation.wav'),('IR sola',root/'rs24_wet_isolation.wav'),('Mezcla final',Path('output/rs24_v10_sweep_revised.wav'))]
fig,ax=plt.subplots(2,1,figsize=(12,7),layout='constrained');summary=[]
for name,p in files:
 fs,x=wavfile.read(p);x=x.astype(float)/32768
 ts=[];low=[]
 for t in np.arange(.5,9.51,.25):
  z=x[int(t*fs):int((t+.5)*fs)];f,v=welch(z,fs,nperseg=8192);ts.append(t+.25);low.append(100*v[f<200].sum()/v.sum())
 ax[0].plot(ts,low,label=name,lw=1.8)
 z=x[3*fs:4*fs];f,v=welch(z,fs,nperseg=8192);v=v/v.sum()
 ax[1].plot(f,10*np.log10(np.maximum(v,1e-12)),label=name,lw=1.3)
 summary.append({'name':name,'bass_fraction_3to4s_percent':float(100*v[f<200].sum()),'top_frequency_3to4s_hz':float(f[np.argmax(v)])})
ax[0].set(xlabel='Tiempo (s)',ylabel='Energía bajo 200 Hz (%)',ylim=(0,100),title='El grave aparece en la convolución');ax[0].legend()
ax[1].set(xlim=(0,2000),ylim=(-100,-15),xlabel='Frecuencia (Hz)',ylabel='Densidad relativa (dB)',title='Espectro entre 3 y 4 s, cerca de 8600 RPM');ax[1].axvline(8600/24,c='cyan',ls='--',lw=1,label='Cadencia por bancada 358 Hz');ax[1].axvline(8600/12,c='grey',ls='--',lw=1,label='Cadencia total 717 Hz');ax[1].legend()
fig.savefig(root/'bass_origin.png',dpi=140)
for item in summary:print(item)
