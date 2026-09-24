from pathlib import Path
import json,hashlib,numpy as np
from scipy.io import wavfile
from scipy.signal import welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
old=Path('output/rs24_v10_sweep_revised.wav');new=Path('output/rs24_v10_sweep_bass_tamed.wav');root=Path('output/analysis_rs24');series=[];stats={}
fig,axs=plt.subplots(2,1,figsize=(11,7),layout='constrained')
for p,label,color in [(old,'Antes: IR originales, mezcla 60 %','tab:red'),(new,'Ahora: IR filtradas a 150 Hz, mezcla 50 %','tab:blue')]:
 fs,x=wavfile.read(p);a=x.astype(float)/32768;times=[];bass=[]
 for t in np.arange(.5,9.51,.25):
  z=a[int(t*fs):int((t+.5)*fs)];f,s=welch(z,fs,nperseg=8192);times.append(t+.25);bass.append(100*s[f<200].sum()/s.sum())
 axs[0].plot(times,bass,lw=1.8,label=label,color=color)
 z=a[3*fs:4*fs];f,s=welch(z,fs,nperseg=8192);axs[1].plot(f,10*np.log10(np.maximum(s,1e-16)),lw=1.4,label=label,color=color)
 rms=np.sqrt(np.mean(a*a));stats[p.name]={'sample_rate':fs,'duration_s':len(a)/fs,'rms_normalized':float(rms),'peak_i16':int(max(abs(x.astype(np.int32)))),'pcm_clipped_samples':int(np.count_nonzero(abs(x.astype(np.int32))>=32767)),'bass_under_200_3to4s_percent':float(100*s[f<200].sum()/s.sum()),'subbass_under_100_3to4s_percent':float(100*s[f<100].sum()/s.sum())}
axs[0].set(xlabel='Tiempo (s)',ylabel='Energía bajo 200 Hz (%)',ylim=(0,80),title='Grave durante el barrido');axs[0].legend()
axs[1].set(xlim=(0,1500),xlabel='Frecuencia (Hz)',ylabel='Densidad espectral (dBFS/Hz)',title='Espectro entre 3 y 4 s (~8600 RPM)');axs[1].axvline(8600/24,color='grey',ls='--',lw=1,label='Cadencia por bancada ~358 Hz');axs[1].legend()
fig.savefig(root/'bass_tamed_comparison.png',dpi=145)
meta=json.loads((root/'processed_ir_metadata.json').read_text());stats['rms_difference_db']=float(20*np.log10(stats[new.name]['rms_normalized']/stats[old.name]['rms_normalized']));stats['source_irs_unchanged']=all(hashlib.sha256(Path(item['source']).read_bytes()).hexdigest()==item['source_sha256'] for item in meta['processed_files']);cfg=json.loads(Path('engines/rs24_v10_sweep_bass_tamed.json').read_text());stats['configuration']={'convolution':cfg['audio']['convolution'],'volume':cfg['audio']['volume'],'impulse_responses':[e['impulse_response'] for e in cfg['exhausts']]};(root/'bass_tamed_metrics.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))
