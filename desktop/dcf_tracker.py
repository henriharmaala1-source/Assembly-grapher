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
    def _sample(self, f, cx, cy, region, n=None):
        """Bilinear resample of a square region to n x n, edge-clamped."""
        N = self.N if n is None else n
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

    def _feats(self, frame, cx, cy, region, n=None):
        n = self.N if n is None else n
        out = []
        y = self._sample(frame['y'], cx, cy, region, n)
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
                a = self._sample(frame[c], cx, cy, region, n)
            else:
                raise ValueError(c)
            a = a - a.mean()
            nrm = np.sqrt((a * a).sum()) + 1e-5
            w = self.win if a.shape == self.win.shape else _hann2(a.shape[0])
            out.append((a / nrm) * w)      # windowed to kill FFT wrap edges
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

    # -- wide re-detection ------------------------------------------------
    #
    # A SECOND filter trained at a larger padding, rather than zero-padding the
    # narrow filter up to a bigger grid.
    #
    # The padding approach was tried first and abandoned: embedding an NxN
    # filter into an nxn grid has to preserve the FFT origin through
    # fftshift/pad/ifftshift, and three plausible conventions all failed a
    # self-test on known displacements (170 px error, or a constant 55 px bias
    # -- displacement tracked, origin wrong). Correct FFT bookkeeping was not
    # worth more time when a second filter costs 1.8 ms and cannot be subtly
    # wrong: it is the SAME code path as the narrow one, just with a wider
    # region, so it is verified by everything that already verifies that path.
    #
    # Why a separate filter rather than reusing the narrow one over a bigger
    # region: the filter learns the target at a fixed pixels-per-target ratio.
    # Feed it a 3x wider region resampled to the same grid and the target
    # appears 3x smaller than anything it was trained on, and it simply does
    # not match -- a wide search that silently finds nothing.
    # MEASURED OUTCOME of using this to rescue the classical tracker: net zero.
    #
    #     clip                   NCC   re-designate   relocate only
    #     d_pan_shake             71%          95%            69%
    #     g_occlusion             79%          76%            81%
    #     h_clutter_distractor    99%          77%           100%
    #     MEAN                    70%          69%            70%
    #
    # The re-detection itself works -- self-tested to +-80px against the narrow
    # filter's +-40px, with PSR separating found (9-11) from lost (3.0-3.6). What
    # fails is the HANDOVER, and not for the reason first assumed.
    #
    # First hypothesis: a wider search finds the wrong object. So the accepted
    # peak was gated on proximity to the coasted motion prediction. That changed
    # NOTHING -- identical results at 1.0/1.5/2.5 box radii, including settings
    # tight enough to reject most of the search window. The re-detections were
    # never far away and the diagnosis was wrong.
    #
    # Actual cause: handing over with designate() REBUILDS templates, anchors
    # and keyframes from the current frame -- which is by definition the frame
    # where tracking had just failed. With a distractor overlapping, the new
    # appearance model is contaminated even though the position barely moved.
    # Repositioning instead fixes exactly that (h_clutter 77% -> 100%,
    # g_occlusion 76% -> 81%) and destroys the case that worked (d_pan_shake
    # 95% -> 69%), because there the appearance HAS changed and rebuilding is
    # what made the rescue succeed.
    #
    # Both handovers are right half the time. What decides it is whether the
    # target still looks like its old template -- exactly what neither variant
    # knows at the moment it must choose.
    #
    # THIRD ATTEMPT: supply that verdict. The frozen anchors are the one thing
    # in the tracker that cannot have drifted, so NCC against them scores
    # whether the target still LOOKS like the designation. Low similarity ->
    # the target changed, rebuild; high -> the model is fine, relocate.
    #
    #     always rebuild  always relocate  verdict .30  .45  .60
    #         69%              70%             67%      67%  67%
    #
    # Worse than either fixed mode, at every threshold. And the per-clip pattern
    # shows it is not a tuning problem: d_pan_shake 69% (chose relocate, needed
    # rebuild) and h_clutter 77% (chose rebuild, needed relocate) -- backwards on
    # BOTH, and identical at all three thresholds, so the signal is not even
    # varying within a clip.
    #
    # The reason is structural. LOW anchor similarity means either "the target
    # changed appearance" or "this is not the target" -- and those two need
    # OPPOSITE actions. One patch of evidence cannot separate them, so no
    # threshold on it can work. Inverting the rule just reproduces one of the two
    # fixed modes.
    #
    # Three attempts, three negatives: the wide re-detector is not the way in on
    # this battery. What makes the network gate reach 93% is that the network is
    # a better RE-FINDER, and re-finding well is what removes the need to decide
    # about templates at all.
    def make_wide(self, frame, cx, cy, size, padding=6.0):
        w = DCFTracker(channels=self.chan, size=self.N, padding=padding,
                       lr=self.lr, lam=self.lam, scales=(1.0,))
        w.init(frame, cx, cy, size)
        return w
