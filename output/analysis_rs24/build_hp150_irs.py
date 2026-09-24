from pathlib import Path
import json,hashlib,numpy as np
from scipy.io import wavfile
from scipy.signal import butter,sosfilt
src=Path('assets/sound-library/smooth');dst=Path('assets/sound-library/rs24');dst.mkdir(parents=True,exist_ok=True)
meta={'filter':'causal Butterworth high-pass','order':4,'cutoff_hz':150,'processed_files':[]}
for stem in ('smooth_01','smooth_02'):
 p=src/(stem+'.wav');fs,x=wavfile.read(p);sos=butter(4,150,btype='highpass',fs=fs,output='sos');y=sosfilt(sos,x.astype(np.float64));peak=float(np.max(np.abs(y)));scale=min(1.0,32767.0/peak);out=np.rint(y*scale).clip(-32768,32767).astype(np.int16);q=dst/(stem+'_hp150.wav');wavfile.write(q,fs,out)
 meta['processed_files'].append({'source':str(p),'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'output':str(q),'source_peak':int(np.max(np.abs(x.astype(np.int32)))),'filtered_peak_before_scale':peak,'scale':scale,'output_peak':int(np.max(np.abs(out.astype(np.int32))))})
print(json.dumps(meta,indent=2));Path('output/analysis_rs24/processed_ir_metadata.json').write_text(json.dumps(meta,indent=2))
