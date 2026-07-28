#!/usr/bin/env python3
"""
Multi-channel discriminative correlation filter — the thing the CSRT ablation
actually pointed at.

Why this and not the simpler idea
---------------------------------
Ablating CSRT showed its entire 18-point lead over this project's tracker is a
COLOUR DESCRIPTOR that preserves hue (+11 points), while the parts it is famous
for contribute nothing: the ADMM spatial reliability map 0, channel reliability
weights 0, HOG +1, the 33-scale search +1.

The obvious cheap conclusion was "then just stop discarding hue" — carry U and V
as separate NCC cues instead of collapsing them to chroma magnitude. That was
tried first and it FAILED, decisively:

    FUSE3 (current) 70%   HUE 60%   HUE2 58%   HUE4 55%   HUEmag 60%

Every hue variant was WORSE. So the colour descriptor and the discriminative
filter are not separable, and that is the whole point of this file. Extra colour
channels inside a template matcher are extra noise: NCC only ever learns what
the target looks like, so a weak, chroma-subsampled, analog-blurred U channel
contributes spurious peaks and the PSR-weighted fusion has no way to know the
channel is worthless. A DCF learns from the same patch what the BACKGROUND looks
like too — every cyclic shift is an implicit negative example — so a channel
that cannot separate target from background simply gets a filter near zero and
stops contributing. The discriminative step is what makes colour safe to add.

The method
----------
Ridge regression in the Fourier domain, MOSSE's running-average form extended to
C channels (the standard multi-channel translation filter, as in DSST/KCF):

    A_c <- (1-lr) A_c + lr * (G  * conj(F_c))
    B   <- (1-lr) B   + lr * (sum_c F_c * conj(F_c))
    response = real(ifft( sum_c Z_c * A_c / (B + lambda) ))

One inverse FFT yields the response at EVERY position at once, which is the
structural advantage over the current search: measured, 11 channels costs ~6x the
arithmetic of the present 3-cue NCC while evaluating 16384 candidate positions
instead of 225.
"""
import numpy as np


def _hann2(n):
    w = np.hanning(n)
    return np.outer(w, w).astype(np.float32)


class DCFTracker:
    """Translation DCF over configurable feature channels.

    channels:
      'y'   log-compressed luma      -- the classic MOSSE channel
      'u'   chroma U (signed)        -- HUE, the thing chroma magnitude destroys
      'v'   chroma V (signed)
      'g'   gradient magnitude       -- cheap stand-in for HOG (+1 pt in CSRT)
    """

    def __init__(self, channels=('y', 'u', 'v'), size=64, padding=2.0,
                 lr=0.025, lam=1e-3, sigma_factor=1 / 16.0, scales=(0.97, 1.0, 1.03)):
        self.chan = tuple(channels)
        self.N = int(size)
        self.padding = float(padding)
        self.lr = float(lr)
        self.lam = float(lam)
        self.win = _hann2(self.N)
        s = self.N * sigma_factor
        yy, xx = np.mgrid[0:self.N, 0:self.N]
        c = (self.N - 1) / 2.0
        g = np.exp(-((xx - c) ** 2 + (yy - c) ** 2) / (2 * s * s)).astype(np.float32)
        # Peak at the ORIGIN in the FFT sense, so a response peak at index (0,0)
        # means "no shift" and the wrap-around search is symmetric.
        self.G = np.fft.fft2(np.fft.ifftshift(g))
        self.scales = tuple(scales)
        self.A = None
        self.B = None
        self.cx = self.cy = 0.0
        self.size = 0.0
        self.score = 0.0

    # -- features ---------------------------------------------------------
    def _sample(self, f, cx, cy, region):
        """Bilinear resample of a square region to N x N, edge-clamped."""
        N = self.N
        ax = np.linspace(cx - region / 2, cx + region / 2, N, dtype=np.float32)
        ay = np.linspace(cy - region / 2, cy + region / 2, N, dtype=np.float32)
        H, W = f.shape
        x0 = np.clip(np.floor(ax), 0, W - 1).astype(np.int32)
        y0 = np.clip(np.floor(ay), 0, H - 1).astype(np.int32)
        x1 = np.clip(x0 + 1, 0, W - 1); y1 = np.clip(y0 + 1, 0, H - 1)
        tx = np.clip(ax - x0, 0, 1); ty = np.clip(ay - y0, 0, 1)
        top = f[np.ix_(y0, x0)] * (1 - tx) + f[np.ix_(y0, x1)] * tx
        bot = f[np.ix_(y1, x0)] * (1 - tx) + f[np.ix_(y1, x1)] * tx
        return (top * (1 - ty[:, None]) + bot * ty[:, None]).astype(np.float32)

    def _feats(self, frame, cx, cy, region):
        out = []
        y = self._sample(frame['y'], cx, cy, region)
        for c in self.chan:
            if c == 'y':
                # log compression: flattens the huge dynamic range an AGC'd
                # analog chain produces, so one bright patch cannot dominate.
                a = np.log(np.maximum(y, 0) + 1.0)
            elif c == 'g':
                gx = np.zeros_like(y); gy = np.zeros_like(y)
                gx[:, 1:-1] = y[:, 2:] - y[:, :-2]
                gy[1:-1, :] = y[2:, :] - y[:-2, :]
                a = np.sqrt(gx * gx + gy * gy)
            elif c in ('u', 'v'):
                a = self._sample(frame[c], cx, cy, region)
            else:
                raise ValueError(c)
            a = a - a.mean()
            n = np.sqrt((a * a).sum()) + 1e-5
            out.append((a / n) * self.win)      # windowed to kill FFT wrap edges
        return out

    # -- API --------------------------------------------------------------
    def init(self, frame, cx, cy, size):
        self.cx, self.cy, self.size = float(cx), float(cy), float(max(4.0, size))
        F = [np.fft.fft2(a) for a in self._feats(frame, self.cx, self.cy,
                                                 self.size * self.padding)]
        self.A = [self.G * np.conj(f) for f in F]
        self.B = sum(f * np.conj(f) for f in F).real
        self.score = 1.0

    def _respond(self, frame, cx, cy, region):
        Z = [np.fft.fft2(a) for a in self._feats(frame, cx, cy, region)]
        num = sum(z * a for z, a in zip(Z, self.A))
        r = np.real(np.fft.ifft2(num / (self.B + self.lam * self.B.size)))
        return r, Z

    def update(self, frame):
        if self.A is None:
            return None
        # Scale is searched by re-sampling the same filter at a few region sizes;
        # the response is comparable because every channel is unit-normalised.
        best = None
        for sc in self.scales:
            region = self.size * self.padding * sc
            r, Z = self._respond(frame, self.cx, self.cy, region)
            pk = float(r.max())
            if best is None or pk > best[0]:
                best = (pk, r, Z, sc, region)
        pk, r, Z, sc, region = best

        iy, ix = np.unravel_index(np.argmax(r), r.shape)
        # PSR on the response, same confidence notion the rest of the project uses
        mask = np.ones_like(r, bool)
        y0, y1 = max(0, iy - 5), min(r.shape[0], iy + 6)
        x0, x1 = max(0, ix - 5), min(r.shape[1], ix + 6)
        mask[y0:y1, x0:x1] = False
        side = r[mask]
        self.score = float((pk - side.mean()) / (side.std() + 1e-6))

        # wrap to signed shift, then convert filter px -> frame px
        dy = iy - self.N if iy > self.N // 2 else iy
        dx = ix - self.N if ix > self.N // 2 else ix
        k = region / self.N
        self.cx += dx * k
        self.cy += dy * k
        self.size *= sc

        # adapt at the CORRECTED position
        F = [np.fft.fft2(a) for a in self._feats(frame, self.cx, self.cy,
                                                 self.size * self.padding)]
        for i, f in enumerate(F):
            self.A[i] = (1 - self.lr) * self.A[i] + self.lr * (self.G * np.conj(f))
        self.B = (1 - self.lr) * self.B + self.lr * sum(f * np.conj(f) for f in F).real
        return self.cx, self.cy, self.size
