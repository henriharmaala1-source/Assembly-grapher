#!/usr/bin/env python3
"""
P2-A — MOSSE correlation filter, benchmarked against NCC as a MATCHER.

The tracker's per-frame cost ceiling is the brute-force NCC search:
O(positions x template^2). A MOSSE-style adaptive correlation filter does the
whole-window search in O(N log N) via the FFT, adapts every frame, and yields
PSR natively. This script isolates the matcher — bare single-scale MOSSE vs bare
single-scale NCC, same crop sequence — and reports localization accuracy + an
op-count estimate, so the swap decision is measured, not guessed.

The Kotlin port is track/Mosse.kt (its own radix-2 FFT — no numpy). Default
tracker matcher stays NCC until this is confirmed on real footage (P0-B).
"""
import numpy as np
import simtrack as st

SZ = 64                      # MOSSE window (power of 2)
TMPL = st.TMPL               # NCC template (28)


# ---------------- MOSSE ----------------
class Mosse:
    def __init__(self, sz=SZ, sigma=2.0, eta=0.125, lam=1e-2):
        self.sz = sz; self.eta = eta; self.lam = lam
        ax = np.arange(sz) - sz // 2
        gx, gy = np.meshgrid(ax, ax)
        g = np.exp(-(gx * gx + gy * gy) / (2 * sigma * sigma))   # peak at centre
        self.G = np.fft.fft2(g)
        wx = np.hanning(sz)
        self.win = np.outer(wx, wx)                              # cosine window (edge damping)

    def _pre(self, patch):
        p = np.log(np.maximum(patch, 0) + 1.0)
        p = (p - p.mean()) / (p.std() + 1e-5)
        return p * self.win

    def init(self, patch):
        f = self._pre(patch); F = np.fft.fft2(f)
        self.A = self.G * np.conj(F)
        self.B = F * np.conj(F) + self.lam

    def track(self, patch):
        """Return (dx,dy) shift of the target from the patch centre, plus PSR.
        Response = ifft2(Z ⊙ H*), H* = A/B. With g peaked at the patch centre the
        response peaks at the target's position, so the peak IS the location."""
        z = self._pre(patch); Z = np.fft.fft2(z)
        r = np.real(np.fft.ifft2(Z * (self.A / self.B)))
        pk = np.unravel_index(np.argmax(r), r.shape)
        psr = self._psr(r, pk)
        dy = pk[0] - self.sz // 2; dx = pk[1] - self.sz // 2
        return float(dx), float(dy), psr

    def update(self, patch):
        f = self._pre(patch); F = np.fft.fft2(f)
        self.A = self.eta * (self.G * np.conj(F)) + (1 - self.eta) * self.A
        self.B = self.eta * (F * np.conj(F) + self.lam) + (1 - self.eta) * self.B

    @staticmethod
    def _psr(r, pk):
        peak = r[pk]; m = np.ones_like(r, bool)
        y0, y1 = max(0, pk[0] - 5), pk[0] + 6; x0, x1 = max(0, pk[1] - 5), pk[1] + 6
        m[y0:y1, x0:x1] = False
        side = r[m]
        return float((peak - side.mean()) / (side.std() + 1e-6))


# ---------------- bare NCC matcher (isolated, single template/scale) ----------
def ncc_track(tmpl, tn, crop, cx, cy, search=22, stride=3):
    r, gs = st.ncc_map(crop, tmpl, tn, cx - search, cx + search, stride)   # square grid centred at cx
    # ncc_map uses one axis range for both; recompute centred properly:
    return r, gs


def run_matcher(scenario, which):
    frames, gt = scenario()
    g0 = gt[0]
    bsize = 44.0
    # designate
    cx0, cy0 = g0
    if which == 'mosse':
        crop0 = st.resample(frames[0]['y'], cx0 - SZ / 2, cy0 - SZ / 2, SZ, SZ, SZ, SZ)
        m = Mosse(); m.init(crop0)
    else:
        r = bsize * st.MARGIN
        c0 = st.crop_raw(frames[0], cx0, cy0, bsize)
        tmpl = st.norm_patch(c0['y'], st.CROP / 2, st.CROP / 2, TMPL); tn = st.nrm(tmpl)
    px, py = cx0, cy0
    errs = []
    for i in range(1, len(frames)):
        gx, gy = gt[i]
        if which == 'mosse':
            crop = st.resample(frames[i]['y'], px - SZ / 2, py - SZ / 2, SZ, SZ, SZ, SZ)
            dx, dy, psr = m.track(crop)
            px += dx; py += dy
            crop2 = st.resample(frames[i]['y'], px - SZ / 2, py - SZ / 2, SZ, SZ, SZ, SZ)
            m.update(crop2)
        else:
            r = bsize * st.MARGIN
            crop = st.resample(frames[i]['y'], px - r / 2, py - r / 2, r, r, st.CROP, st.CROP)
            c0 = st.CROP // 2; sh = 22
            resp, _ = st.ncc_map(crop, tmpl, tn, c0 - sh, c0 + sh, 3)
            sx, sy = st.subpix(resp)
            cxc = (c0 - sh) + sx * 3; cyc = (c0 - sh) + sy * 3
            px += (cxc / st.CROP - 0.5) * r; py += (cyc / st.CROP - 0.5) * r
            # adapt
            c2 = st.resample(frames[i]['y'], px - r / 2, py - r / 2, r, r, st.CROP, st.CROP)
            fr = st.norm_patch(c2, st.CROP / 2, st.CROP / 2, TMPL)
            tmpl = 0.92 * tmpl + 0.08 * fr; tn = st.nrm(tmpl)
        errs.append(np.hypot(px - gx, py - gy))
    errs = np.array(errs)
    return errs.mean(), np.percentile(errs, 90), 100 * (errs < 25).mean()


def opcount():
    # NCC (anchor+adaptive): 2 templates x grid positions x template^2 real mults
    gw = (2 * 22) // 3 + 1
    ncc = 2 * gw * gw * TMPL * TMPL
    # MOSSE: fft2 = 2*sz 1D FFTs each way; track = ~3 fft2 (Z, ifft, +preprocess);
    # update = 2 fft2. Count complex mults ~ sz^2 * log2(sz^2) per fft2.
    fft2 = SZ * SZ * np.log2(SZ * SZ)
    mosse = 5 * fft2 * 6   # ~5 fft2/frame, 6 real-mults per complex op (rough)
    return int(ncc), int(mosse), gw


if __name__ == '__main__':
    print(f"MOSSE(sz={SZ}) vs NCC matcher — bare single-scale, per scenario\n")
    print(f"{'scenario':<11}{'matcher':<8}{'mean':>7}{'p90':>7}{'lock%':>7}")
    for sname in ('translate', 'fast', 'pan', 'approach'):
        for which in ('ncc', 'mosse'):
            # fresh seed per matcher so both see identical frames
            st.rng = np.random.RandomState(99); st.BGT = st.rng.rand(240, 320).astype(np.float32) * 40 + 60
            m, p90, lk = run_matcher(st.SCEN[sname], which)
            print(f"{sname:<11}{which:<8}{m:7.1f}{p90:7.1f}{lk:7.0f}")
        print()
    ncc, mosse, gw = opcount()
    ncc_full = 2 * SZ * SZ * TMPL * TMPL     # NCC covering the SAME 64x64 area, dense
    print(f"per-frame op estimate (rough):")
    print(f"  NCC  (2 tmpl x {gw}x{gw} grid x {TMPL}^2)        = {ncc:,}   [searches {gw*gw} positions]")
    print(f"  MOSSE (FFT, full {SZ}x{SZ} dense search)         = {mosse:,}   [searches {SZ*SZ} positions]")
    print(f"  NCC to match MOSSE's {SZ}x{SZ} coverage densely  = {ncc_full:,}")
    print(f"  -> for EQUAL search coverage, MOSSE is ~{ncc_full/mosse:.1f}x cheaper than NCC —")
    print(f"     the frame-cost headroom for a bigger re-acquire search (P0-A).")
