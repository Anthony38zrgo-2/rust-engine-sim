//! Diagnostic: analyze a WAV for clipping, loudness and spectral structure.
//! Usage: cargo run -p es-config --bin analyze -- <file.wav>

use es_audio_io::read_wav_mono_i16;
use rustfft::num_complex::Complex;
use rustfft::FftPlanner;

#[derive(Default, Clone)]
struct WindowStats {
    peak: f64,
    sum2: f64,
    zcr: usize,
    at_target: usize,
    hi: usize,
    n: usize,
    prev: f64,
}

fn top_peaks(bins: &[f64], count: usize) -> Vec<(f64, f64)> {
    let mut idx: Vec<usize> = (4..bins.len()).collect();
    idx.sort_by(|&a, &b| bins[b].partial_cmp(&bins[a]).unwrap());
    let mut out: Vec<(f64, f64)> = Vec::new();
    let sr = 44100.0f64;
    let fft_n = 4096.0;
    for &i in &idx {
        if out.len() >= count {
            break;
        }
        if out.iter().any(|&(f, _)| ((f - i as f64 * sr / fft_n).abs()) < 25.0) {
            continue;
        }
        out.push((i as f64 * sr / fft_n, bins[i]));
    }
    out
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "output/rs24_v10_sweep.wav".into());
    let (sr, samples) = match read_wav_mono_i16(&path) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read error: {e}");
            std::process::exit(1);
        }
    };
    let sr = sr as usize;
    let n = samples.len();
    println!(
        "file {path}: {n} samples @ {sr} Hz ({:.2}s)",
        n as f64 / sr as f64
    );

    let secs = n / sr;
    let mut sec_stats: Vec<WindowStats> = vec![WindowStats::default(); secs];
    let mut g = WindowStats::default();
    for (i, &s) in samples.iter().enumerate() {
        let v = s as f64;
        let a = v.abs();
        let w = &mut sec_stats[i / sr];
        for st in [&mut g, w] {
            st.n += 1;
            if a > st.peak {
                st.peak = a;
            }
            st.sum2 += v * v;
            if (v >= 0.0) != (st.prev >= 0.0) {
                st.zcr += 1;
            }
            st.prev = v;
            if a >= 30000.0 {
                st.at_target += 1;
            }
            if a > 15000.0 {
                st.hi += 1;
            }
        }
    }
    println!(
        "{:>3} {:>8} {:>8} {:>6} {:>9} {:>9} {:>8}",
        "s", "peak", "rms", "crest", "zcr_khz", "pin30000", "hi50%fs"
    );
    for (s, w) in sec_stats.iter().enumerate() {
        let rms = (w.sum2 / w.n as f64).sqrt();
        let crest = if rms > 0.0 { w.peak / rms } else { 0.0 };
        println!(
            "{:>3} {:>8.0} {:>8.0} {:>6.2} {:>9.1} {:>8.2}% {:>7.1}%",
            s + 1,
            w.peak,
            rms,
            crest,
            w.zcr as f64 / (w.n as f64 / sr as f64) / 1000.0,
            100.0 * w.at_target as f64 / w.n as f64,
            100.0 * w.hi as f64 / w.n as f64,
        );
    }

    // Spectra: 4096-sample Hann windows at 25% / 50% / 85% of the file.
    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(4096);
    for &frac in &[0.25f64, 0.5, 0.85] {
        let start = (((n as f64 * frac) as usize).saturating_sub(2048)).min(n.saturating_sub(4096));
        let mut buf: Vec<Complex<f64>> = Vec::with_capacity(4096);
        for i in 0..4096 {
            let w = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / 4095.0).cos();
            let v = samples[start + i] as f64 / 32768.0;
            buf.push(Complex::new(v * w, 0.0));
        }
        fft.process(&mut buf);
        let bins: Vec<f64> = buf[..2048].iter().map(|c| c.norm()).collect();
        println!("spectrum @ t={:.2}s:", start as f64 / sr as f64);
        for (f, m) in top_peaks(&bins, 6) {
            println!("    {:7.1} Hz  mag={:.4}", f, m);
        }
    }
}
