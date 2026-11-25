"""Gyro noise: spectra, a throttle (or RPM) vs frequency map, and peak classification.

The key question for frame design is whether a noise peak is
  * structural: stays at a fixed frequency whatever the throttle  -> frame/arm resonance
  * motor/prop: moves up with throttle / RPM                       -> rotational excitation
Peaks are tracked across throttle bins and classified by how their frequency behaves.
"""

from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks, spectrogram, welch

from .config import Aircraft
from .loader import AXES, Log, contiguous_blocks

N_BINS = 10


def _welch_blocks(x: np.ndarray, fs: float, blocks, nperseg: int):
    acc, wsum, f = None, 0, None
    for a, b in blocks:
        seg = x[a:b]
        seg = seg[np.isfinite(seg)]
        if seg.size < nperseg:
            continue
        f, p = welch(seg - seg.mean(), fs=fs, nperseg=nperseg)
        acc = p * seg.size if acc is None else acc + p * seg.size
        wsum += seg.size
    if acc is None:
        return None, None
    return f, acc / wsum


def _hf_rms(f, p, fmin=20.0):
    sel = f >= fmin
    return float(np.sqrt(np.trapezoid(p[sel], f[sel])))


def analyze(log: Log, ac: Aircraft, phases: dict) -> dict:
    df, fs = log.df, log.fs
    flying = df["mavg"].to_numpy() > 0.05 if "mavg" in df else np.ones(len(df), bool)
    t = df["t"].to_numpy()
    blocks = [(a, b) for a, b in contiguous_blocks(t, fs, 2.0) if flying[a:b].mean() > 0.8]
    if not blocks:
        return {"available": False, "reason": "no continuous flight long enough for spectra"}
    nper = int(2 ** np.floor(np.log2(fs / 2)))       # ~0.5 s windows, ~2 Hz resolution
    res: dict = {"available": True, "fs": fs, "axes": {}}

    for i, ax in enumerate(AXES):
        raw_col = f"gr{i}" if f"gr{i}" in df else (f"gy{i}" if f"gy{i}" in df else None)
        if raw_col is None:
            continue
        f, praw = _welch_blocks(df[raw_col].to_numpy(), fs, blocks, nper)
        if f is None:
            continue
        entry = {"f": f.tolist(), "raw": praw.tolist(), "raw_source": raw_col, "rms_raw_hf": _hf_rms(f, praw)}
        if f"gy{i}" in df and raw_col != f"gy{i}":
            _, pf = _welch_blocks(df[f"gy{i}"].to_numpy(), fs, blocks, nper)
            entry["filt"] = pf.tolist()
            entry["rms_filt_hf"] = _hf_rms(f, pf)
            entry["filter_attenuation_db"] = float(20 * np.log10(entry["rms_raw_hf"] / max(1e-9, entry["rms_filt_hf"])))
        db = 10 * np.log10(praw + 1e-12)
        band = (f >= 20) & (f <= 0.9 * fs / 2)
        pk, props = find_peaks(db[band], prominence=6, distance=max(1, int(5 / (f[1] - f[0]))))
        order = np.argsort(props["prominences"])[::-1][:5]
        entry["peaks_hz"] = [float(f[band][pk[j]]) for j in order]
        if f"d{i}" in df:
            _, pd_ = _welch_blocks(df[f"d{i}"].to_numpy(), fs, blocks, nper)
            if pd_ is not None:
                entry["dterm_hf_fraction"] = float(pd_[f > 100].sum() / max(1e-12, pd_[f > 0].sum()))
        res["axes"][ax] = entry

    # ---- throttle (or RPM) vs frequency map, raw gyro summed over roll+pitch+yaw
    use_rpm = ("rpm_mean" in df and np.nanmedian(df["rpm_mean"]) > 0) or \
              ("erpm_mean" in df and np.nanmedian(df["erpm_mean"]) > 0)
    if use_rpm:
        if "rpm_mean" in df and np.nanmedian(df["rpm_mean"]) > 0:
            xvar = df["rpm_mean"].to_numpy()                                    # already mechanical RPM
        else:
            xvar = df["erpm_mean"].to_numpy() * 100.0 / (ac.motor_poles / 2.0)  # eRPM/100 -> mechanical RPM
        xlabel = "motor RPM"
    else:
        xvar = df["mavg"].to_numpy()
        xlabel = "mean motor output"
    raw_cols = [f"gr{i}" if f"gr{i}" in df else f"gy{i}" for i in range(3) if (f"gr{i}" in df or f"gy{i}" in df)]
    nseg = int(2 ** np.floor(np.log2(fs / 4)))       # ~0.25 s
    spec_sum, xs, fr = [], [], None
    for a, b in blocks:
        tot = None
        for c in raw_cols:
            x = np.nan_to_num(df[c].to_numpy()[a:b])
            fr, tt, S = spectrogram(x - x.mean(), fs=fs, nperseg=nseg, noverlap=nseg // 2)
            tot = S if tot is None else tot + S
        centres = (tt * fs).astype(int)
        spec_sum.append(tot)
        xs.append(np.array([np.nanmean(xvar[a + max(0, c - nseg // 2): a + c + nseg // 2]) for c in centres]))
    S = np.concatenate(spec_sum, axis=1)
    X = np.concatenate(xs)
    ok = np.isfinite(X)
    S, X = S[:, ok], X[ok]
    lo, hi = np.percentile(X, 0.5), np.percentile(X, 99.5)
    edges = np.linspace(lo, hi, N_BINS + 1)
    heat, centres_x = [], []
    for e0, e1 in zip(edges[:-1], edges[1:]):
        sel = (X >= e0) & (X <= e1)
        if sel.sum() >= 2:
            heat.append(S[:, sel].mean(axis=1))
            centres_x.append(0.5 * (e0 + e1))
    if len(heat) >= 3:
        H = np.array(heat).T                         # freq x bins
        res["map"] = {"f": fr.tolist(), "x": centres_x, "xlabel": xlabel, "db": (10 * np.log10(H + 1e-12)).tolist()}
        res["peak_classes"] = _classify(fr, H, np.array(centres_x), fs, use_rpm, ac)
    return res


def _bin_peaks(f, H, fs):
    band = (f >= 30) & (f <= 0.9 * fs / 2)
    fb = f[band]
    out = []
    for k in range(H.shape[1]):
        db = 10 * np.log10(H[band, k] + 1e-12)
        pk, _ = find_peaks(db, prominence=4, distance=max(1, int(8 / (f[1] - f[0]))))
        out.append(fb[pk])
    return out


def _classify(f, H, xbins, fs, use_rpm, ac):
    """Classify dominant peaks.

    structural: a peak at (nearly) the same frequency appears in most load bins.
    motor/prop: starting from the bin where the peak is strongest, follow the nearest peak
                bin by bin; if its frequency rises with load it is rotational.
    """
    total = H.mean(axis=1)
    band = (f >= 30) & (f <= 0.9 * fs / 2)
    fb = f[band]
    pk, props = find_peaks(10 * np.log10(total[band] + 1e-12), prominence=4,
                           distance=max(1, int(8 / (f[1] - f[0]))))
    per_bin = _bin_peaks(f, H, fs)
    nb = H.shape[1]
    x_span = float((xbins.max() - xbins.min()) / max(1e-9, np.mean(xbins)))
    if x_span < 0.3:
        return [{"freq_hz": float(fb[pk[j]]), "class": "unclear (throttle range too narrow to separate frame from motor noise)",
                 "track_hz": [None] * nb, "variation_pct": 0.0, "corr_with_x": 0.0}
                for j in np.argsort(props["prominences"])[::-1][:3]]
    candidates = [float(fb[pk[j]]) for j in np.argsort(props["prominences"])[::-1][:5]]

    def present(f0, tol=0.06):
        return np.array([np.any(np.abs(p - f0) <= tol * f0) for p in per_bin])

    structural = [f0 for f0 in candidates if present(f0).mean() >= 0.7]
    out, tracks = [], []
    for f0 in candidates:
        if f0 in structural:
            track = np.full(nb, f0)
            item = {"freq_hz": f0, "class": "structural (fixed frequency - frame/arm resonance candidate)",
                    "track_hz": track.tolist(), "presence_pct": round(100 * present(f0).mean()),
                    "variation_pct": 0.0, "corr_with_x": 0.0}
        else:
            idx = int(np.argmin(np.abs(f - f0)))
            k0 = int(np.argmax(H[idx, :]))
            track = np.full(nb, np.nan)
            track[k0] = f0
            for direction in (1, -1):
                prev, k = f0, k0 + direction
                while 0 <= k < nb:
                    c = per_bin[k]
                    c = c[(np.abs(c - prev) <= 0.3 * prev) &
                          ~np.any([np.abs(c - s) <= 0.06 * s for s in structural] or [np.zeros(c.size, bool)], axis=0)]
                    if c.size == 0:
                        break
                    prev = float(c[np.argmin(np.abs(c - prev))])
                    track[k] = prev
                    k += direction
            ok = np.isfinite(track)
            if ok.sum() >= 3 and np.std(track[ok]) > 0:
                r = float(np.corrcoef(xbins[ok], track[ok])[0, 1])
                spread = float((track[ok].max() - track[ok].min()) / np.mean(track[ok]))
            else:
                r, spread = 0.0, 0.0
            if r > 0.7 and spread > 0.15:
                kind = "motor/prop (rises with " + ("RPM" if use_rpm else "throttle") + ")"
            else:
                kind = "unclear"
            # the same moving peak is often found from several starting points: keep one
            dup = any(np.nanmean(np.abs(track - t) <= 0.06 * np.abs(t)) > 0.6 for t in tracks if np.isfinite(t).any())
            if dup:
                continue
            tracks.append(track)
            item = {"freq_hz": f0, "class": kind, "track_hz": [None if not np.isfinite(v) else float(v) for v in track],
                    "variation_pct": round(100 * spread, 1), "corr_with_x": round(r, 2)}
        if use_rpm:
            tr = np.array([np.nan if v is None else v for v in item["track_hz"]], float)
            ratios = tr / (xbins / 60.0)
            item["harmonic"] = round(float(np.nanmedian(ratios)), 2)
        out.append(item)
    return out
