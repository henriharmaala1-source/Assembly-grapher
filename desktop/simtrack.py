#!/usr/bin/env python3
"""Faithful Python simulation of the Kotlin LockTracker (cue fusion, sub-pixel,
adaptive search, scale, alpha-beta filter, coasting). Runs challenging synthetic
scenarios with known ground truth to measure tracking quality and drive fixes.
Whatever wins here gets ported back to LockTracker.kt."""
import numpy as np

import os
# Perf params overridable via env for A/B. Smaller template + coarser stride +
# tighter base search cut NCC cost (~positions x template^2) several-fold; the
# sub-pixel parabolic refine covers the coarser grid.
CROP   = int(os.environ.get('CROP', 128))
TMPL   = int(os.environ.get('TMPL', 28))     # matches LockTracker.kt
MARGIN = 2.2
SEARCH = int(os.environ.get('SEARCH', 22))
STRIDE = int(os.environ.get('STRIDE', 3))
SCALES = [0.9, 1.0, 1.11]
LOSS_TIMEOUT, TMPL_EMA = 45, 0.08
FOV_DELAY = 6                                 # coasting frames before search zooms out
K_KEYFRAMES = 2                               # P1-B: diverse-pose keyframes beyond anchor+adaptive
KF_THRESH = 0.55
KF_ADD_CONF = 0.80                            # bank a keyframe only on a very clean lock (anti-contamination)
PSR_LOCK = float(os.environ.get('PSR_LOCK', 5.5))
PSR_WARN = float(os.environ.get('PSR_WARN', 3.8))
SIZE_FLOOR = 36.0                             # matches LockTracker.kt (anti over-zoom)
OCC_FRAC = 0.55                               # P2-B: PSR below this x clean-baseline = occluded
OCC_ENTER = 2                                 # consecutive low frames before declaring occlusion
OCC_MAX = 20                                  # after this many, re-baseline (not an occluder)
ACQUIRE_OPEN = int(os.environ.get('ACQUIRE_OPEN', 0))   # measured: widening at designate HURT (10%->1%)
# Both of these are MEASURED-AND-REJECTED experiments, kept only so the result is
# reproducible. Default OFF. On realistic footage (SNR-calibrated, 1/f background):
#   ACQUIRE_OPEN=5  -> f_maneuver 10%->1%   (a wide window at designate false-locks
#                      on self-similar background rather than finding a fast target)
#   VEL_FOV_K=6     -> g_occlusion 79%->32% (velocity-scaled FOV wrecks occlusion
#                      recovery; mean across hard clips 53%->45%)
VEL_FOV_K = float(os.environ.get('VEL_FOV_K', 0.0))   # FOV growth per unit speed   # frames after designate to search wide
USE_AFFINE_EGO = False
NBEST = 1                # experiment switch: >1 enables N-best peak selection   # experiment switch; see ego_affine_at
EARLY_TERM_PSR = 10.0                         # skip remaining cues once one is this dominant

# ---------------------------------------------------------------- LK coasting
# When the appearance match fails, the tracker coasts on a constant-velocity
# guess: it stops looking at the image entirely and extrapolates. That is fine
# for two or three frames and wrong for twenty -- a manoeuvring target's whole
# problem is that its velocity is NOT constant, which is exactly when the match
# is most likely to have failed in the first place.
#
# Sparse optical flow gives image evidence that costs nothing appearance-wise:
# corners inside the last good box, tracked forward by Lucas-Kanade, median
# displacement. It does not care what the target looks like, so it survives the
# pose change / partial occlusion / motion blur that broke the template.
#
# MEASURED, over the whole real-footage battery (eval_lkcoast.py):
#
#     clip                    base    with LK
#     d_pan_shake              71%        69%     consistent small loss
#     f_maneuver               19%        68%     the reason this exists
#     g_occlusion              79%        90%
#     e_recede                100%       100%
#     MEAN                     70%        77%
#
# Read the mean with care. Sweeping LK_MAX_PTS from 6 to 30 gives means of
# 72,72,69,77,74,67,74,68,75 -- jagged, no plateau, because changing the point
# set changes WHICH frame re-locks and the trajectory diverges from there. The
# honest expected value is the average over that sweep, about +2 points, not the
# +7 the best column shows. What IS consistent, and is why this ships enabled, is
# the SIGN per clip: f_maneuver improves at 8 of 9 point counts (median 39% vs
# 19%) and g_occlusion at 7 of 9 (median 90% vs 79%), while d_pan_shake loses
# 2-7 at every single one. Flow helps when the appearance model fails on a moving
# or occluded target and mildly hurts when the camera shakes, which is what the
# mechanism predicts.
#
# Cost is not measurable against the NCC: 18.8 vs 19.1 ms/frame.
LK_ASSIST   = int(os.environ.get('LK_ASSIST', 1))
# Not resolvable by this battery -- see the jaggedness above. 12 is chosen for
# cost (the median is over 12 displacements, not 30) from a range where every
# value beat the baseline, NOT because 12 scored highest.
LK_MAX_PTS  = int(os.environ.get('LK_MAX_PTS', 12))
LK_MIN_PTS  = int(os.environ.get('LK_MIN_PTS', 4))   # fewer surviving corners = no verdict
LK_QUALITY  = float(os.environ.get('LK_QUALITY', 0.01))   # goodFeaturesToTrack threshold, rel. to best corner
# The four settings below are GATES THAT WERE TRIED AND LOST, kept switchable so
# the result stays reproducible. Every one was invented to fix e_recede, which
# the ungated prototype dropped from 100% to 82%; every one cost more elsewhere
# than it recovered:
#     +inner 0.7  68%     +fb 1.5  69%     +spread 2.5  63%     +minbox 28  73%
# against 73% ungated and 75% with the seed-scope fix alone. e_recede's loss was
# never a missing gate: it was the SEED SCOPE (see LK_FULLSEED). Fix the cause and
# every gate becomes a pure cost.
LK_INNER    = float(os.environ.get('LK_INNER', 1.0))      # seed from the INNER fraction of the box
LK_MIN_BOX  = float(os.environ.get('LK_MIN_BOX', 0.0))    # box side below which corners are background
LK_FB_MAX   = float(os.environ.get('LK_FB_MAX', 1e9))     # forward-backward round-trip ceiling, px (>=1e6 = off)
LK_SPREAD   = float(os.environ.get('LK_SPREAD', 1e9))     # median |d - median(d)| ceiling, px
# Re-seed on EVERY locked frame. Every 4th measured 69% against 75%, and the
# reason is that the flow reference frame only advances when the points do: a
# stale seed means tracking from a frame several old at the moment the target is
# already hard to follow. The seed is a corner detection on a ~60 px sub-image,
# so there is nothing to save by skipping it.
LK_REFRESH  = int(os.environ.get('LK_REFRESH', 1))
# Design switches, kept explicit because the prototype that motivated this
# feature differed from the obvious integration in three ways at once, and the
# obvious integration measured WORSE. See eval_lkcoast.py for the sweep.
# 0 = flow REPLACES the const-vel + ego prediction step
# 1 = flow ADDS to it (double-counts camera motion, which is wrong on paper)
# 2 = flow corrects the OUTPUT after the search has already decided, on any
#     frame that did not lock. Modes 0 and 1 both feed the prediction, so their
#     correction is thrown away whenever the NCC then accepts a peak -- which on
#     a clip that alternates LOCKED/COASTING is most of the time. Mode 2 is what
#     the prototype did, and the difference is 14% vs 62% on f_maneuver.
LK_MODE     = int(os.environ.get('LK_MODE', 2))
LK_VFB      = int(os.environ.get('LK_VFB', 0))      # feed the flow displacement back into alpha-beta velocity
LK_DROP     = int(os.environ.get('LK_DROP', 0))     # 1 = a rejected verdict also kills the point set
# The flow window has to cover a frame's worth of motion or the pyramid cannot
# find the point at all. f_maneuver moves ~18 px/frame, so a 15 px window at 2
# levels -- which looked like a sane cheap default -- simply cannot track it.
LK_WIN      = int(os.environ.get('LK_WIN', 21))
LK_LEVELS   = int(os.environ.get('LK_LEVELS', 3))
# Seed scope. goodFeaturesToTrack's qualityLevel is relative to the STRONGEST
# CORNER IN THE IMAGE IT IS GIVEN, so the same number means two different things
# on a full frame under a box mask (relative to the best corner in the scene --
# strict) and on a cropped sub-image (relative to the best corner in the box --
# permissive, and on a low-texture target that is whatever noise is present).
# The sub-image is ~60x cheaper AND it is the better of the two: 75% against 73%,
# and it is what recovers e_recede from 82% to 100%. The mechanism is the reverse
# of the guess above. On a shrinking target a full-frame threshold keeps only the
# corners that are strong relative to the WHOLE SCENE, and those are the
# high-contrast background edges clipped by the box, not the target -- so flow
# then measures the background's motion and glues the box to the scene while the
# target recedes out of it. Box-relative selection keeps the best corners ON THE
# TARGET, which is what the assist needs, and is also the cheap option.
LK_FULLSEED = int(os.environ.get('LK_FULLSEED', 0))
# Seed the point set from the designation itself, so a target lost on the very
# first frame still has flow available. Sounds obviously right; measured, it
# costs -- the designation box is the user's rectangle, not a fitted one, so its
# corners are the least target-specific the tracker will ever hold.
LK_SEED0    = int(os.environ.get('LK_SEED0', 0))
# 'lk'  = OpenCV Shi-Tomasi + pyramidal Lucas-Kanade (what the sweep measured)
# 'ssd' = grid of textured patches + brute-force SSD block match, which is what
#         LockTracker.kt can actually run: it is pure Kotlin on GrayFrame with no
#         OpenCV, deliberately, because the same algorithm has to survive the port
#         to the onboard C++. OpticalFlow.kt already implements exactly this
#         matcher (early-exit SSD, variance gate, forward-backward check) for
#         ego-motion, so the port would reuse proven code -- but it is a
#         DIFFERENT algorithm from the one measured above, so it was measured too
#         rather than assumed equivalent.
#
#         IT IS NOT EQUIVALENT, and the reason is worth the port's attention:
#
#             three hard clips        f_maneuver  g_occlusion  e_recede   mean
#             OpenCV LK                      68%          90%      100%    86%
#             SSD block match, best          22%          83%      100%    68%
#             OpenCV LK, output ROUNDED      44%          44%      100%    63%
#
#         Two fixes to the portable matcher worked exactly as diagnosed and were
#         kept: a Shi-Tomasi min-eigenvalue gate in place of the variance gate
#         took e_recede from 63% to 100% (variance accepts edge patches that
#         slide, min-eigenvalue rejects them), and a coarse-to-fine pyramid
#         replaced a wide flat search at a tenth the arithmetic. Neither
#         recovered f_maneuver.
#
#         The third row is why. Rounding the OpenCV displacement to whole pixels
#         -- changing nothing else -- reproduces the block matcher's score. So
#         what the assist depends on is SUB-PIXEL displacement, not the matching
#         method: over a hundred coasting frames, half-pixel rounding errors
#         integrate into a drift larger than the target.
#
#         A block matcher cannot supply that, so the port must implement the
#         ITERATIVE LK refinement (structure tensor, d <- d + G^-1 b) rather than
#         reuse OpticalFlow.kt's search. That is still pure arithmetic and still
#         MCU-portable -- it is just not the code that already exists.
LK_METHOD   = os.environ.get('LK_METHOD', 'lk')
SSD_PATCH   = int(os.environ.get('SSD_PATCH', 5))    # half-size of the match patch
SSD_SEARCH  = int(os.environ.get('SSD_SEARCH', 12))  # half-size of the search window
SSD_MINVAR  = float(os.environ.get('SSD_MINVAR', 40.0))
# Patch selection for the portable variant. 'var' is what OpticalFlow.kt already
# uses; 'eig' is the Shi-Tomasi min-eigenvalue of the structure tensor, which is
# what OpenCV's goodFeaturesToTrack uses. The difference is the aperture problem:
# a patch straddling a strong straight EDGE has high variance but is only
# localisable across the edge, not along it, so a block match slides freely and
# reports a displacement that is partly arbitrary. min-eigenvalue is small for
# exactly those patches. It costs three gradient sums per candidate.
SSD_GATE    = os.environ.get('SSD_GATE', 'eig')
SSD_MINEIG  = float(os.environ.get('SSD_MINEIG', 8.0))
SSD_LEVELS  = int(os.environ.get('SSD_LEVELS', 3))   # pyramid levels for the portable matcher

# Temporal averaging of the TRACKING CROP (0/1 = off, else the window length).
# MEASURED AND REJECTED, default off, kept so the result is reproducible:
# 70% -> 64/63/58% at N=2/3/4 (eval_tavg_pip.py). It is a genuinely different
# experiment from the frame-level averaging that also lost -- see the comment at
# the point of use -- and it loses for a different reason and nearly as much.
TAVG_PIP    = int(os.environ.get('TAVG_PIP', 0))
TAVG_TOL    = 0.05         # fractional regionw change that invalidates the buffer

def resample(a, rx, ry, rw, rh, oW, oH):
    H, W = a.shape
    ys = np.clip(ry + (np.arange(oH)+0.5)*(rh/oH), 0, H-1)
    xs = np.clip(rx + (np.arange(oW)+0.5)*(rw/oW), 0, W-1)
    y0 = ys.astype(int); y1 = np.minimum(y0+1, H-1); ty = (ys-y0)[:,None]
    x0 = xs.astype(int); x1 = np.minimum(x0+1, W-1); tx = (xs-x0)[None,:]
    A = a[np.ix_(y0,x0)]; B = a[np.ix_(y0,x1)]; C = a[np.ix_(y1,x0)]; D = a[np.ix_(y1,x1)]
    top = A+(B-A)*tx; bot = C+(D-C)*tx
    return (top+(bot-top)*ty).astype(np.float32)

def sobel(g):
    gx = np.zeros_like(g); gy = np.zeros_like(g)
    gx[1:-1,1:-1] = (g[:-2,2:]+2*g[1:-1,2:]+g[2:,2:])-(g[:-2,:-2]+2*g[1:-1,:-2]+g[2:,:-2])
    gy[1:-1,1:-1] = (g[2:,:-2]+2*g[2:,1:-1]+g[2:,2:])-(g[:-2,:-2]+2*g[:-2,1:-1]+g[:-2,2:])
    return np.minimum(np.hypot(gx,gy), 255).astype(np.float32)

def apply_cue(crop, cue):
    if cue == 'none': return crop['y']
    if cue == 'edge': return sobel(crop['y'])
    if cue == 'chroma': return np.minimum(np.hypot(crop['u'],crop['v'])*1.41,255).astype(np.float32)
    # HUE-PRESERVING cues. 'chroma' above is chroma MAGNITUDE -- one dimension,
    # which separates saturated from unsaturated and cannot tell red from blue.
    # Ablating OpenCV's CSRT showed its entire 18-point advantage over this
    # tracker is a colour descriptor that keeps hue (+11 points), while the
    # famous parts -- the ADMM spatial reliability map, channel weights, HOG,
    # the 33-scale search -- contribute 0, 0, +1 and +1. So the cheap version of
    # that finding is simply to stop throwing hue away: U and V carried as
    # SEPARATE signed channels span the chroma plane instead of collapsing it to
    # a radius. NCC is zero-mean normalised, so the signed range is fine as-is.
    if cue == 'cu': return crop['u'].astype(np.float32)
    if cue == 'cv': return crop['v'].astype(np.float32)
    # Opponent pair rotated 45 deg: catches hues that lie along a U or V axis and
    # would be near-constant in one of the two channels above.
    if cue == 'co1': return ((crop['u'] + crop['v']) * 0.7071).astype(np.float32)
    if cue == 'co2': return ((crop['u'] - crop['v']) * 0.7071).astype(np.float32)
    return crop['y']

def ms(a): return a - a.mean()
def nrm(a): return np.sqrt((a*a).sum())+1e-6

def crop_raw(frame, cx, cy, size):
    r = size*MARGIN
    return {k: resample(frame[k], cx-r/2, cy-r/2, r, r, CROP, CROP) for k in ('y','u','v')}

def norm_patch(chan, cx, cy, sz):
    p = resample(chan, cx-sz/2, cy-sz/2, sz, sz, sz, sz)
    return ms(p)

def ncc_map(chan, tmpl, tn, g0, g1, stride=STRIDE):
    """Zero-mean NCC over a grid of candidate positions, all at once.

    This used to be a Python double loop calling numpy on a 28x28 patch per
    position. Profiling the viewer put ncc_map at 10.3 s of 13 s, with ~300,000
    calls to ndarray.mean and the norm helper -- i.e. essentially all of it was
    numpy's PER-CALL overhead, not arithmetic. The tracker is not slow; calling
    numpy 300,000 times is.

    Vectorised via a strided view, using the same identity as the Kotlin
    nccAt: with a zero-mean template, sum((v-mean)*t) == sum(v*t), and
    sum((v-mean)^2) == sum(v^2) - sum(v)^2/N. So no patch ever needs centring
    and the whole grid is three tensor reductions. Bit-comparable to the loop,
    verified against it.
    """
    gs = np.arange(g0, g1+1, stride); gw = len(gs)
    h = TMPL // 2
    H, W = chan.shape
    resp = np.full((gw, gw), -2.0, np.float32)
    ok = (gs - h >= 0) & (gs + TMPL - h <= W) & (gs + TMPL - h <= H)
    if not ok.any():
        return resp, gs
    win = np.lib.stride_tricks.sliding_window_view(chan, (TMPL, TMPL))
    ys = gs[ok] - h
    P = win[np.ix_(ys, ys)].astype(np.float32)          # (n,n,TMPL,TMPL)
    # CENTRE THE PATCHES EXPLICITLY. The tempting one-pass form -- keep sum(v)
    # and sum(v^2) and use sum((v-mean)^2) == sum(v^2) - sum(v)^2/n -- is what
    # the Kotlin nccAt does, and it is unstable here. Both terms are ~5e7 for
    # 8-bit pixels over a 28x28 patch while their difference is the variance,
    # which on a low-contrast patch is many orders smaller: catastrophic
    # cancellation, and the denominator comes out wrong precisely on flat
    # regions. Real crops are full of them (sky, saturated ground); random test
    # data is not, which is why the one-pass version passed synthetic checks at
    # 3e-08 and was still off by 9.4 on real frames after the numerator was
    # fixed. Centring costs one extra pass over an array that is already in
    # cache, and it is exact.
    Pc = P - P.mean((2, 3), keepdims=True)
    num = np.einsum('ijkl,kl->ij', Pc, tmpl.astype(np.float32), dtype=np.float64)
    den = np.sqrt(np.einsum('ijkl,ijkl->ij', Pc, Pc, dtype=np.float64))
    r = (num / (tn * (den + 1e-6))).astype(np.float32)
    idx = np.nonzero(ok)[0]
    resp[np.ix_(idx, idx)] = r
    return resp, gs

def psr_of(resp):
    pk = np.unravel_index(np.argmax(resp), resp.shape); peak = resp[pk]
    m = np.ones_like(resp, bool)
    y0,y1 = max(0,pk[0]-3), pk[0]+4; x0,x1 = max(0,pk[1]-3), pk[1]+4; m[y0:y1,x0:x1]=False
    side = resp[m]
    if side.size < 4: return 0.0
    return float((peak - side.mean())/(side.std()+1e-6))

def subpix(resp):
    pk = np.unravel_index(np.argmax(resp), resp.shape); peak = resp[pk]
    py,px = pk; dy=dx=0.0
    if 0<px<resp.shape[1]-1:
        l,r = resp[py,px-1], resp[py,px+1]; den=l-2*peak+r
        if abs(den)>1e-6: dx = np.clip(0.5*(l-r)/den,-1,1)
    if 0<py<resp.shape[0]-1:
        u,d = resp[py-1,px], resp[py+1,px]; den=u-2*peak+d
        if abs(den)>1e-6: dy = np.clip(0.5*(u-d)/den,-1,1)
    return px+dx, py+dy

def psr2conf(p): return np.clip((p-3)/9, 0, 1)

class Tracker:
    def __init__(self, cues, latency=4.5):
        self.cues=cues; self.latency=latency
        self.tmpl=None; self.state='IDLE'; self.bad=0; self.conf=0
        self.lkpts=None; self.lkgrey=None; self.lkage=0; self.lkused=0
    def designate(self, frame, px, py, size):
        self.bcx=px; self.bcy=py; self.bsize=float(np.clip(size,SIZE_FLOOR,min(frame['y'].shape)))
        crop=crop_raw(frame,px,py,self.bsize)
        self._build(crop)
        self.x=px; self.y=py; self.vx=0.0; self.vy=0.0
        self.bad=0; self.conf=1; self.state='LOCKED'; self.psrema=0.0; self.prevY=frame['y']; self.occluded=False; self.occlow=0; self.since=0
        self.lkpts=None; self.lkgrey=None; self.lkage=0; self.lkused=0
        if LK_ASSIST and LK_SEED0:
            self._seed(np.clip(frame['y'],0,255).astype(np.uint8))
    def _build(self, crop):
        self.tmpl=[]; self.tn=[]
        for c in self.cues:
            t=norm_patch(apply_cue(crop,c), CROP/2, CROP/2, TMPL)
            self.tmpl.append(t); self.tn.append(nrm(t))
        # fixed ANCHORS = the original views (anti-drift; used alone in wide search).
        self.anchor=[t.copy() for t in self.tmpl]; self.an=list(self.tn)
        # P1-B appearance bank: per cue, extra diverse-pose KEYFRAMES (slots 2..K),
        # added only when a confident view is sufficiently DIFFERENT from every
        # stored slot. Response = max over {anchor, adaptive, keyframes}. Holds a
        # target through pose/lighting swings that a lone EMA template smears over,
        # and can't drift (each keyframe is a real observed view).
        self.kf=[[] for _ in self.cues]; self.kfn=[[] for _ in self.cues]
        self.tl=norm_patch(crop['y'], CROP/2, CROP/2, TMPL); self.tln=nrm(self.tl)
        self.histfg, self.histbg = hist_counts_at(crop, CROP/2, CROP/2)   # STAPLE-style cue

    # ---- LK coasting assist ------------------------------------------------
    def _seed(self, grey):
        (self._ssd_seed if LK_METHOD=='ssd' else self._lk_seed)(grey)

    def _step(self, grey):
        return (self._ssd_step if LK_METHOD=='ssd' else self._lk_step)(grey)

    # Portable variant: no OpenCV, so LockTracker.kt / the onboard C++ can run
    # the same thing. Corner selection becomes a patch-VARIANCE gate over a grid
    # (flat patches are ambiguous to match, which is the same property Shi-Tomasi
    # tests) and pyramidal LK becomes a brute-force SSD search over +-SSD_SEARCH.
    def _ssd_seed(self, grey):
        self.lkage=0
        h=self.bsize*0.5*LK_INNER
        H,W=grey.shape; m=SSD_PATCH+SSD_SEARCH
        x0=int(np.clip(self.bcx-h,m,W-m-1)); x1=int(np.clip(self.bcx+h,m,W-m-1))
        y0=int(np.clip(self.bcy-h,m,H-m-1)); y1=int(np.clip(self.bcy+h,m,H-m-1))
        if x1-x0<4 or y1-y0<4: self.lkpts=None; return
        g=max(2,int(np.sqrt(LK_MAX_PTS))+1)
        xs=np.linspace(x0,x1,g).astype(np.int32); ys=np.linspace(y0,y1,g).astype(np.int32)
        p=SSD_PATCH; cand=[]
        for cy in ys:
            for cx in xs:
                w_=grey[cy-p:cy+p+1, cx-p:cx+p+1].astype(np.float32)
                if SSD_GATE=='eig':
                    gx=0.5*(w_[1:-1,2:]-w_[1:-1,:-2]); gy=0.5*(w_[2:,1:-1]-w_[:-2,1:-1])
                    a=float((gx*gx).mean()); b=float((gy*gy).mean()); c=float((gx*gy).mean())
                    tr=a+b; det=a*b-c*c
                    v=0.5*(tr-np.sqrt(max(0.0,tr*tr-4*det)))   # min eigenvalue
                    if v<SSD_MINEIG: continue
                else:
                    v=float(w_.var())
                    if v<SSD_MINVAR: continue
                cand.append((v,cx,cy))
        if len(cand)<LK_MIN_PTS: self.lkpts=None; return
        # Strongest-textured first, so a small point budget spends itself on the
        # patches that can actually be matched unambiguously.
        cand.sort(key=lambda t:-t[0])
        pts=np.array([[[c[1],c[2]]] for c in cand[:LK_MAX_PTS]],np.float32)
        self.lkpts=pts; self.lkgrey=grey

    @staticmethod
    def _pyr(g, n):
        """2x2 box-average pyramid. Decimation and an add are all the onboard
        side needs; no filter kernel, no interpolation."""
        out=[g.astype(np.float32)]
        for _ in range(n-1):
            a=out[-1]; h=(a.shape[0]//2)*2; w=(a.shape[1]//2)*2
            if h<16 or w<16: break
            out.append(0.25*(a[0:h:2,0:w:2]+a[1:h:2,0:w:2]+a[0:h:2,1:w:2]+a[1:h:2,1:w:2]))
        return out

    def _ssd_match(self, pa, pb, cx, cy):
        """Coarse-to-fine block match, returning level-0 displacement or None.

        This is the ingredient a flat block search cannot substitute for. On a
        target crossing ~18 px per frame with the motion blur that comes with it,
        a single-scale match has to search +-20 px (1681 candidate positions) and
        still fails, because at full resolution the blurred target no longer
        matches its own sharp template. Halving twice turns 18 px of motion into
        4.5 px and blur into structure. It is also CHEAPER: ~5 px of search at
        the coarsest level plus two +-2 refinements is ~12k operations per point
        against ~200k for the flat search it replaces.
        """
        p=SSD_PATCH; L=len(pa)-1; dx=dy=0
        for lv in range(L,-1,-1):
            sc=1<<lv; a=pa[lv]; b=pb[lv]
            px=int(round(cx/sc)); py=int(round(cy/sc))
            rad=max(1,int(np.ceil(SSD_SEARCH/float(sc)))) if lv==L else 2
            if px-p<0 or py-p<0 or px+p>=a.shape[1] or py+p>=a.shape[0]: return None
            t=a[py-p:py+p+1, px-p:px+p+1]
            x0=px+dx-p-rad; y0=py+dy-p-rad
            x1=px+dx+p+rad+1; y1=py+dy+p+rad+1
            if x0<0 or y0<0 or x1>b.shape[1] or y1>b.shape[0]: return None
            win=np.lib.stride_tricks.sliding_window_view(b[y0:y1, x0:x1],(2*p+1,2*p+1))
            diff=win-t
            ssd=np.einsum('ijkl,ijkl->ij',diff,diff)
            iy,ix=np.unravel_index(np.argmin(ssd),ssd.shape)
            dx+=ix-rad; dy+=iy-rad
            if lv>0: dx*=2; dy*=2
        return dx,dy

    def _ssd_step(self, grey):
        if self.lkpts is None or self.lkgrey is None or len(self.lkpts)<LK_MIN_PTS:
            return None
        pa=self._pyr(self.lkgrey,SSD_LEVELS); pb=self._pyr(grey,SSD_LEVELS)
        d=[]; keep=[]
        for q in self.lkpts:
            cx,cy=int(round(float(q[0,0]))),int(round(float(q[0,1])))
            r=self._ssd_match(pa,pb,cx,cy)
            if r is None: continue
            d.append(r); keep.append((cx+r[0],cy+r[1]))
        if len(d)<LK_MIN_PTS: self.lkpts=None; return None
        d=np.asarray(d,np.float32)
        self.lkpts=np.asarray(keep,np.float32).reshape(-1,1,2); self.lkgrey=grey
        self.lkused+=1
        return float(np.median(d[:,0])), float(np.median(d[:,1]))

    def _lk_seed(self, grey):
        """Re-detect corners inside the current box, in frame coordinates.

        goodFeaturesToTrack runs on the CROPPED sub-image, not on the full frame
        with a mask. Masking still computes the eigenvalue map everywhere, which
        is the entire cost -- on a 60 px box this is ~3600 px of corner response
        instead of ~230000. It also changes WHICH corners come back, and that
        turns out to matter more than the speed: see LK_FULLSEED.
        """
        import cv2
        self.lkage=0; self.lkfail=0
        if self.bsize < LK_MIN_BOX:
            # Below this the box holds too few pixels for corner detection to
            # find anything but noise; whatever it returns will be background.
            self.lkpts=None; return
        h=self.bsize*0.5*LK_INNER
        H,W=grey.shape
        x0=int(np.clip(self.bcx-h,0,W-1)); x1=int(np.clip(self.bcx+h,x0+8,W))
        y0=int(np.clip(self.bcy-h,0,H-1)); y1=int(np.clip(self.bcy+h,y0+8,H))
        if LK_FULLSEED:
            m=np.zeros_like(grey); m[y0:y1, x0:x1]=255
            p=cv2.goodFeaturesToTrack(grey, LK_MAX_PTS, LK_QUALITY, 3, mask=m)
            x0=y0=0
        else:
            sub=grey[y0:y1, x0:x1]
            if sub.shape[0]<12 or sub.shape[1]<12:
                self.lkpts=None; return
            p=cv2.goodFeaturesToTrack(sub, LK_MAX_PTS, LK_QUALITY, 3)
        if p is None or len(p)<LK_MIN_PTS:
            # Not enough structure to flow. Saying "no verdict" is the whole
            # point: a weak-corner target is one where LK returns the motion of
            # whatever texture happens to be nearby, and coasting on the
            # constant-velocity prediction is strictly better than that.
            self.lkpts=None; return
        p[:,0,0]+=x0; p[:,0,1]+=y0
        self.lkpts=p.astype(np.float32); self.lkgrey=grey; self.lkage=0

    def _lk_step(self, grey):
        """Median LK displacement of the seeded corners, or None.

        The quality gates this was built around are all DISABLED by default,
        because measuring them one at a time showed every one costs more than it
        recovers (see the LK_INNER block above). They stay implemented and
        switchable so that result stays reproducible, and because the reasoning
        behind them is not wrong in general -- it was wrong here:

          fwd-bwd  TLD's round-trip test, and the most respectable of the four.
                   It rejects points that slid along an edge or latched onto an
                   occluder. On this battery it also rejects the points that
                   carry a fast target through a blur, and f_maneuver goes
                   62% -> 19%. A coasting tracker has nothing else; a
                   half-trustworthy displacement beats extrapolation.
          spread   requires the survivors to agree, on the theory that a mix of
                   target and background gives two clusters and the median lands
                   between them. Worst of the four (63%): during exactly the
                   partial occlusion this feature is for, the points SHOULD
                   disagree, and refusing to answer then is refusing to help.
          inner    seed from the middle of the box only, to avoid background.
                   Made redundant by box-relative corner selection, and on a
                   small target it starves the detector.
          min box  never fired on any clip -- identical results with and without.

        The median over all surviving points is doing the robustness work that
        these gates were meant to add, and it does it without ever withholding an
        answer.
        """
        import cv2
        if self.lkpts is None or self.lkgrey is None or len(self.lkpts)<LK_MIN_PTS:
            return None
        lk=dict(winSize=(LK_WIN,LK_WIN), maxLevel=LK_LEVELS,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        nxt,stf,_=cv2.calcOpticalFlowPyrLK(self.lkgrey, grey, self.lkpts, None, **lk)
        if nxt is None or stf is None:
            self.lkpts=None; return None
        ok=stf.ravel()==1
        if LK_FB_MAX<1e6:                       # >=1e6 disables the round trip entirely
            back,stb,_=cv2.calcOpticalFlowPyrLK(grey, self.lkgrey, nxt, None, **lk)
            if back is None or stb is None:
                self.lkpts=None; return None
            fb=np.hypot(*(back-self.lkpts).reshape(-1,2).T)
            ok=ok&(stb.ravel()==1)&(fb<=LK_FB_MAX)
        if ok.sum()<LK_MIN_PTS:
            # Out of points. This is the only way the flow track dies, and it is
            # a real death: there is nothing left to advance.
            self.lkpts=None; return None
        d=(nxt[ok]-self.lkpts[ok]).reshape(-1,2)
        # ADVANCE THE POINTS FIRST, unconditionally, and only then decide whether
        # to believe the displacement. Rejecting the verdict and the point set
        # together was the first version's mistake and it was silently fatal:
        # three rejected frames dropped the track, so on the clip this feature
        # exists for -- a 100-frame coast through a manoeuvre -- LK ran on 2
        # frames out of 101 and the whole gain vanished. A frame whose corners
        # disagree is a frame with no ANSWER, not a dead track; the points are
        # still on the image and still where the flow says they are.
        self.lkpts=nxt[ok].reshape(-1,1,2).astype(np.float32); self.lkgrey=grey
        mdx=float(np.median(d[:,0])); mdy=float(np.median(d[:,1]))
        if float(np.median(np.hypot(d[:,0]-mdx, d[:,1]-mdy)))>LK_SPREAD:
            if LK_DROP: self.lkpts=None
            return None
        self.lkused+=1
        return mdx,mdy

    def update(self, frame):
        # P1-A ego-motion feed-forward: estimate the camera pan (prev->cur, median
        # grid flow — target rejected as outlier) and add it to the PREDICTION so a
        # pan doesn't push the target out of the crop before the filter catches up.
        # Added to position, not velocity, so vx,vy stay target-relative (no double-
        # count). Gated on grid-flow CONSENSUS (inlier fraction): high on a rigid
        # pan, low under noise / a big occluder — so it's a clean no-op except on a
        # real camera pan. Capped so a bad estimate can't throw the crop.
        edx=edy=0.0
        if EGO and getattr(self,'prevY',None) is not None and self.prevY.shape==frame['y'].shape:
            # exclude the CURRENT box from the flow grid — a large/dominant target's
            # own motion could otherwise win the median vote with high consensus.
            ex=(self.bcx, self.bcy, self.bsize*MARGIN*0.5)
            fx,fy,cons=ego_estimate(self.prevY, frame['y'], ex=ex)
            if cons<EGO_CONS: fx=fy=0.0            # distrust low-consensus flow (noise/occluder)
            # Deadband: sub-EGO_DEAD flow is matching jitter (the NCC search absorbs it).
            if abs(fx)<EGO_DEAD: fx=0.0
            if abs(fy)<EGO_DEAD: fy=0.0
            cap=self.bsize*MARGIN*0.4
            if USE_AFFINE_EGO:
                # Displacement AT THE TARGET rather than one vector for the whole
                # frame. Self-tested against a known warp: under 6% zoom + 3 deg
                # roll the median translation is 11-13 px wrong at the corners
                # while the similarity fit is 0.1 px; on a pure pan both are
                # exact, so it degrades to the old behaviour rather than
                # replacing it.
                aff = ego_affine_at(self.bcx, self.bcy)
                if aff is not None:
                    fx, fy = aff
            edx=float(np.clip(fx,-cap,cap)); edy=float(np.clip(fy,-cap,cap))
        self.prevY=frame['y']
        # LK coast assist. Only while the appearance match is already failing:
        # when the template still matches, the NCC peak is a better position
        # estimate than a median of corner displacements, and letting flow move
        # the prediction under a good lock just injects its own drift.
        lkd=None; grey=None
        if LK_ASSIST and LK_MODE<2 and self.bad>0 and self.lkpts is not None:
            grey=np.clip(frame['y'],0,255).astype(np.uint8)
            lkd=self._step(grey)
        if lkd is None:
            self.x+=self.vx+edx; self.y+=self.vy+edy
        elif LK_MODE==0:
            # Measured displacement REPLACES the constant-velocity step and the
            # ego feed-forward. Image motion IS target motion plus camera motion
            # and LK measures the sum, so adding ego on top would count the pan
            # twice.
            self.x+=lkd[0]; self.y+=lkd[1]
        else:
            self.x+=self.vx+edx+lkd[0]; self.y+=self.vy+edy+lkd[1]
        if lkd is not None and LK_VFB:
            # Feed it back into the alpha-beta velocity so that when LK goes
            # quiet, the coast that resumes runs at the speed last OBSERVED
            # rather than the one from before the target started manoeuvring.
            self.vx=0.5*self.vx+0.5*lkd[0]; self.vy=0.5*self.vy+0.5*lkd[1]
        pcx,pcy=self.x,self.y
        # zoom the search out (see LockTracker.kt) — only AFTER normal coasting has
        # failed for FOV_DELAY frames (early coasting rides the const-vel prediction
        # onto the target; zooming out early hurts). Coarser stride holds cost flat.
        # The FIELD OF VIEW must cover where the target can plausibly be, so it has
        # to scale with SPEED -- not only with elapsed loss frames. Measured on
        # realistic footage (f_maneuver, ~18 px/frame at the 36 px size floor): the
        # crop spans only 79 frame-px, so the target clears the ENTIRE crop 3 frames
        # after a miss, while the loss-driven expansion waits for FOV_DELAY=6. By
        # then it is 108 px away and unrecoverable -- no search-window or threshold
        # tweak inside that crop can help, which is why widening the window and
        # raising PSR_WARN both failed to move this case.
        # Expanding cropPix with fov keeps px-per-world-unit constant, so the
        # template still matches; the coarser stride holds the cost flat.
        speed=np.hypot(self.vx,self.vy)
        fov_vel = float(np.clip(1.0 + VEL_FOV_K*speed/(self.bsize*MARGIN), 1.0, 3.0))
        wide = self.bad >= FOV_DELAY          # 'wide' still means LOST (anchor-only + strict accept)
        fov_lost = min(1+0.3*(self.bad-FOV_DELAY+1), 3.0) if wide else 1.0
        fov = max(fov_vel, fov_lost)
        croppix = (int(CROP*fov)//2)*2 if fov>1 else CROP
        stride_eff = max(1, int(STRIDE*fov))
        regionw = self.bsize*MARGIN*fov
        crop = {k: resample(frame[k], pcx-regionw/2, pcy-regionw/2, regionw, regionw, croppix, croppix)
                for k in ('y','u','v')}
        # Temporal averaging, restricted to the tracking crop.
        #
        # Averaging ego-ALIGNED FULL FRAMES was measured and lost decisively
        # (70% -> 65%, g_occlusion 79% -> 35%). The reason is structural: frame
        # alignment cancels CAMERA motion only, so a moving target is smeared by
        # exactly its own displacement -- the averaging destroys the one thing
        # being matched, and the harder the target moves the more it destroys.
        #
        # The crop does not have that defect, and gets its alignment for free.
        # It is resampled about the tracker's own predicted centre every frame,
        # so a tracked target sits at the SAME crop coordinate in consecutive
        # crops with no warping: residual misalignment is just the prediction
        # error, a couple of pixels, not the target's full displacement. Noise is
        # temporally uncorrelated and averages down by sqrt(N); what smears
        # instead is the BACKGROUND, which for a template matcher trying to
        # separate target from background is a second benefit rather than a cost.
        #
        # Invalidated whenever the crop geometry moves: crops taken at different
        # region widths are the same scene at different scales, and averaging
        # those is just blur.
        #
        # MEASURED: 70% -> 64/63/58% at N=2/3/4. The premise holds and is not
        # enough. Crop alignment is only as good as the PREDICTION it is centred
        # on, which is worst exactly when the target moves, and the crop then
        # magnifies that error: a 36 px box sampled over 79 px into a 128 px crop
        # turns 1 px of frame-space prediction error into ~1.6 px of misalignment
        # against a 28 px template. f_maneuver falls 19% -> 3%.
        if TAVG_PIP>1:
            prev=getattr(self,'_tak',None)
            if prev is None or prev[0]!=croppix or abs(regionw/prev[1]-1.0)>TAVG_TOL:
                self._tak=(croppix,regionw); self._tab=[]
            self._tab.append(crop)
            if len(self._tab)>TAVG_PIP: self._tab.pop(0)
            if len(self._tab)>1:
                crop={k: np.mean([c[k] for c in self._tab],axis=0,dtype=np.float32)
                      for k in ('y','u','v')}
        velcrop=speed*CROP/(self.bsize*MARGIN)
        maxhalf=croppix//2-TMPL//2
        # Search width must track PREDICTION UNCERTAINTY. It is highest right
        # after designate: the alpha-beta filter has no velocity estimate yet, so
        # velcrop is 0 and the window collapses to its base -- only +/-13.6 frame
        # px at the size floor, which a fast target clears on frame ONE (measured
        # on f_maneuver: target moved 17.8 px and was never seen again). Treat the
        # just-designated case like coasting and open up until velocity settles.
        self.since += 1
        wideOpen = self.bad>0 or self.since <= ACQUIRE_OPEN
        half = maxhalf if wideOpen else int(np.clip(SEARCH+velcrop*2,SEARCH,maxhalf))
        c0=croppix//2; g0=c0-half; g1=c0+half
        # FIX 2: ANCHOR-CONSENSUS fusion + conditional prior.
        #  (a) Each cue's response, peak, PSR. The most-confident cue is the
        #      ANCHOR (luma during scale, edge on a same-brightness target — it
        #      adapts). Others are weighted by PSR AND agreement with the anchor,
        #      so a cue that locked on a distractor or drifted under scale (peak
        #      disagrees) is down-weighted. Fixes scale + appearance failures.
        #  (b) Identical distractors can't be told apart by appearance, so a
        #      CONDITIONAL spatial prior (only when a rival peak exists) biases
        #      toward the prediction. Fixes the identical-distractor case.
        fused=None; anyw=0; sig_p=None; cc=None
        for ci,c in enumerate(self.cues):
            chan=apply_cue(crop,c)
            # anchor alone during a wide re-acquire; max over the appearance bank
            # {anchor, adaptive, keyframes} otherwise.
            rb,_=ncc_map(chan,self.anchor[ci],self.an[ci],g0,g1,stride_eff)
            if wide: r=rb
            else:
                ra,_=ncc_map(chan,self.tmpl[ci],self.tn[ci],g0,g1,stride_eff)
                r=np.maximum(ra,rb)
                # Keyframes are a TARGETED fallback, not always-on: consult them only
                # while locked (bad==0, target present — not during occlusion) AND the
                # primary anchor+adaptive response is weak (PSR<lock — the signature
                # of a pose shift the primary can't match). Always-on max just raises
                # the response noise floor and hurt occlusion/noisy cases in sim.
                if self.bad==0 and self.kf[ci] and psr_of(r)<PSR_LOCK:
                    for kf,kfn in zip(self.kf[ci],self.kfn[ci]):
                        rk,_=ncc_map(chan,kf,kfn,g0,g1,stride_eff); r=np.maximum(r,rk)
            gw=r.shape[0]; cc=(gw-1)/2
            if sig_p is None: sig_p = gw/1.4 if self.bad>0 else gw/2.5
            pk=np.unravel_index(np.argmax(r),r.shape)
            # (a) prediction-proximity: a cue whose peak drifts off (confidently-
            #     wrong edge under scale, or a distractor lock) is down-weighted,
            #     so the cue on the predicted target wins. PSR alone can't — a
            #     sharp-but-wrong peak has high PSR.
            prox=np.exp(-(((pk[1]-cc)**2+(pk[0]-cc)**2)/(2*sig_p*sig_p)))
            cue_psr=psr_of(r)
            w=max(0,cue_psr-3)*prox
            if w<=0: continue
            anyw+=w; fused=r*w if fused is None else fused+r*w
            # Early termination: once one cue is already overwhelmingly dominant
            # (well past the lock threshold), the remaining cues' NCC is spent for
            # negligible marginal fusion weight — skip them this frame.
            if cue_psr>EARLY_TERM_PSR: break
        # STAPLE-style histogram cue (chroma fg/bg, no spatial layout — survives
        # deformation/rotation that breaks the spatial NCC cues above). Only
        # during a normal-FOV search (crop is CROP-sized, matching how the fg/bg
        # masks were built); the wide re-acquire stays anchor-NCC-only as before.
        if not wide and getattr(self,'histfg',None) is not None:
            hr=hist_response(crop, self.histfg, self.histbg, g0, g1, stride_eff)
            hgw=hr.shape[0]
            if cc is None: cc=(hgw-1)/2
            if sig_p is None: sig_p = hgw/1.4 if self.bad>0 else hgw/2.5
            hpk=np.unravel_index(np.argmax(hr),hr.shape)
            hprox=np.exp(-(((hpk[1]-cc)**2+(hpk[0]-cc)**2)/(2*sig_p*sig_p)))
            # Damped, STAPLE-style: the histogram's box-summed peak is less
            # spatially precise than a real spatial-NCC peak, so letting it compete
            # on fully equal footing (unbounded self-assessed PSR) let it DOMINATE
            # fusion whenever a spatial cue was merely noisy (not truly occluded) —
            # sim-confirmed regression on noisy_occ/edge (98%->86% lock). Capping its
            # weight keeps it a genuine contributor (still wins the `occlusion`
            # scenario) without letting it override an otherwise-working spatial cue.
            hw=max(0,psr_of(hr)-3)*hprox*HIST_WEIGHT_CAP
            if hw>0:
                anyw+=hw; fused=hr*hw if fused is None else fused+hr*hw
        if anyw>0:
            gw=fused.shape[0]
            # (b) conditional prior for IDENTICAL distractors (appearance can't
            #     separate them; the per-cue weighting above can't either since
            #     both peaks sit inside each cue's response). Suppress a rival
            #     peak on the fused map by biasing to the prediction.
            pk=np.unravel_index(np.argmax(fused),fused.shape); peakv=fused[pk]
            supp=fused.copy(); supp[max(0,pk[0]-4):pk[0]+5,max(0,pk[1]-4):pk[1]+5]=-1e9
            pk2=np.unravel_index(np.argmax(supp),supp.shape)
            if peakv>0.1 and supp[pk2]>0.6*peakv and np.hypot(pk2[0]-pk[0],pk2[1]-pk[1])>5:
                sig=gw/1.5 if self.bad>0 else gw/2.6
                yy,xx=np.mgrid[0:gw,0:gw]
                fused=fused*np.exp(-(((xx-cc)**2+(yy-cc)**2)/(2*sig*sig))).astype(np.float32)
            curpsr=psr_of(fused); self.conf=psr2conf(curpsr)
        else: curpsr=0.0; self.conf=0
        # P2-B occlusion detection: a sharp PSR drop vs the running CLEAN baseline
        # is the occlusion signature (peak collapses, energy spreads). While
        # occluded we still track the visible part for POSITION, but freeze
        # appearance adaptation, keyframe banking and scale — so the template can't
        # drift onto the occluder and wreck recovery. Baseline learns on clean frames.
        # Hysteresis, both ends (the bare threshold was wrong in two ways):
        #  ENTER: a single-frame PSR dip is sensor noise, not an occluder. Require
        #    OCC_ENTER consecutive low frames before freezing adaptation.
        #  EXIT : the baseline could only ratchet UP -- it was only updated while
        #    NOT occluded -- so a target that legitimately gets harder (receding,
        #    fading, low contrast) parks PSR in the band [psrLock, OCC_FRAC*ema]
        #    and stays "occluded" FOREVER, with adaptation and scale frozen for
        #    the rest of the flight. An occlusion is transient by definition; a
        #    lasting drop is a changed target, so after OCC_MAX frames give up and
        #    re-baseline to the new normal.
        low = self.psrema>0 and curpsr < OCC_FRAC*self.psrema
        self.occlow = (self.occlow+1) if low else 0
        if self.occlow > OCC_MAX:
            self.psrema = curpsr          # not an occluder -- this IS the target now
            self.occlow = 0
        occluded = OCC_ENTER <= self.occlow <= OCC_MAX
        if (not occluded) and curpsr>PSR_LOCK:
            self.psrema = curpsr if self.psrema<=0 else 0.9*self.psrema+0.1*curpsr
        self.occluded = occluded
        # re-locking from a wide search demands a strong match (avoid background locks)
        accept = PSR_LOCK if wide else PSR_WARN
        if anyw>0 and self.conf>=psr2conf(accept):
            if NBEST > 1:
                # #3: consider the top-K peaks, not just the highest, and pick
                # the one that best CONTINUES the recent trajectory.
                #
                # Motivated by three failures that all reduce to the same thing:
                # one frame cannot distinguish the target from a similar object
                # beside it. Several frames can -- the distractor does not follow
                # the target's velocity. The existing prediction-proximity term
                # already prefers peaks near the prediction, but it applies to a
                # cue's single peak; this ranks ALTERNATIVES within the fused map.
                f2 = fused.copy(); gwf = fused.shape[0]; cand = []
                for _ in range(NBEST):
                    p2 = np.unravel_index(np.argmax(f2), f2.shape)
                    cand.append((float(f2[p2]), p2))
                    y0 = max(0, p2[0]-2); y1 = min(gwf, p2[0]+3)
                    x0 = max(0, p2[1]-2); x1 = min(gwf, p2[1]+3)
                    f2[y0:y1, x0:x1] = -1e9         # suppress and take the next
                cc2 = (gwf-1)/2.0
                sig = gwf/2.5
                best = None
                for val, p2 in cand:
                    if val <= -1e8: continue
                    d2 = (p2[1]-cc2)**2 + (p2[0]-cc2)**2
                    sc = val * np.exp(-d2/(2*sig*sig))
                    if best is None or sc > best[0]: best = (sc, p2)
                if best is not None and best[1] != tuple(np.unravel_index(np.argmax(fused), fused.shape)):
                    f3 = np.full_like(fused, fused.min())
                    y0 = max(0, best[1][0]-3); y1 = min(gwf, best[1][0]+4)
                    x0 = max(0, best[1][1]-3); x1 = min(gwf, best[1][1]+4)
                    f3[y0:y1, x0:x1] = fused[y0:y1, x0:x1]
                    sx,sy = subpix(f3)
                else:
                    sx,sy=subpix(fused)
            else:
                sx,sy=subpix(fused)
            cxc=g0+sx*stride_eff; cyc=g0+sy*stride_eff
            nx=pcx+(cxc/croppix-0.5)*regionw; ny=pcy+(cyc/croppix-0.5)*regionw
            # correct
            rx=nx-self.x; ry=ny-self.y; self.x+=0.5*rx; self.y+=0.5*ry; self.vx+=0.15*rx; self.vy+=0.15*ry
            # velocity cap: a noisy peak can inject a big residual and the constant-
            # velocity prediction then compounds it until the crop flies off target.
            spd=np.hypot(self.vx,self.vy); vmax=self.bsize*0.9
            if spd>vmax>0: k=vmax/spd; self.vx*=k; self.vy*=k
            self.bcx,self.bcy=self.x,self.y
            if not occluded: self._scale(crop,cxc,cyc)                 # P2-B: hold scale under occlusion
            if self.conf>=psr2conf(PSR_LOCK) and not occluded: self._adapt(crop,cxc,cyc)
            self.state='LOCKED'; self.bad=0
            # Re-seed the corner set on a good lock, periodically rather than
            # every frame: goodFeaturesToTrack is the expensive half of this and
            # the corners only need to be fresh enough that the box they came
            # from is still the right box.
            if LK_ASSIST and LK_MODE<2:
                self.lkage=getattr(self,'lkage',0)+1
                if self.lkpts is None or self.lkage>=LK_REFRESH:
                    if grey is None: grey=np.clip(frame['y'],0,255).astype(np.uint8)
                    self._seed(grey)
        else:
            self.vx*=0.6; self.vy*=0.6          # coast decelerates instead of flying off
            self.bcx,self.bcy=pcx,pcy; self.bad+=1
            self.state=('LOST' if self.bad>=LOSS_TIMEOUT else
                        'SEARCHING' if wide else 'COASTING')
        # LK coast assist, mode 2: correct the OUTPUT of a frame that failed to
        # lock, rather than the prediction that fed the search. The search has
        # already had its say and come back empty; what is reported and carried
        # forward is then pure extrapolation, and this replaces that with
        # measured image displacement. Feeding the same number into the
        # prediction instead (modes 0/1) is nearly a no-op, because any frame
        # where the NCC does accept a peak discards it.
        if LK_ASSIST and LK_MODE==2:
            if self.state!='LOCKED' and self.lkpts is not None:
                if grey is None: grey=np.clip(frame['y'],0,255).astype(np.uint8)
                lkd=self._step(grey)
                if lkd is not None:
                    self.bcx+=lkd[0]; self.bcy+=lkd[1]
                    self.x=self.bcx; self.y=self.bcy
                    if LK_VFB:
                        self.vx=0.5*self.vx+0.5*lkd[0]; self.vy=0.5*self.vy+0.5*lkd[1]
            elif self.state=='LOCKED':
                self.lkage=getattr(self,'lkage',0)+1
                if self.lkpts is None or self.lkage>=LK_REFRESH:
                    if grey is None: grey=np.clip(frame['y'],0,255).astype(np.uint8)
                    self._seed(grey)
        ax=self.x+self.vx*int(self.latency+0.5); ay=self.y+self.vy*int(self.latency+0.5)
        return self.bcx,self.bcy,self.bsize,self.conf,self.state,ax,ay
    def _scale(self,crop,cx,cy):
        chan=crop['y']; t=self.tl; best=-2;bs=1.0; n1=0.0   # FIX 1: scale on luma
        for s in SCALES:
            ts=TMPL*s; p=ms(resample(chan,cx-ts/2,cy-ts/2,ts,ts,TMPL,TMPL))
            n=(p*t).sum()/(nrm(t)*nrm(p))
            if s==1.0: n1=n
            if n>best:best=n;bs=s
        # dead-band: only rescale on a clear win over staying put, else feed noise
        # ratchets the box down to the floor every frame (over-zoom, unstable lock).
        if bs!=1.0 and best<n1+0.03: bs=1.0
        self.bsize=float(np.clip(self.bsize*(1+(bs-1)*0.5),SIZE_FLOOR,min(crop['y'].shape[0]*4,2000)))
    def _adapt(self,crop,cx,cy):
        for ci,c in enumerate(self.cues):
            fr=norm_patch(apply_cue(crop,c),cx,cy,TMPL); fn=nrm(fr)
            # EMA-update the adaptive slot (fast recent appearance), as before.
            self.tmpl[ci]=(1-TMPL_EMA)*self.tmpl[ci]+TMPL_EMA*fr; self.tn[ci]=nrm(self.tmpl[ci])
            # P1-B: bank this view as a KEYFRAME iff (1) the fused lock is VERY clean
            # (conf>=KF_ADD_CONF and no recent miss — excludes partial-occlusion /
            # ambiguous views that would poison the bank) AND (2) it's a genuinely
            # new appearance (below KF_THRESH similarity to every stored slot, so the
            # bank stays diverse, not full of near-duplicates). Evict oldest when full.
            if self.conf>=KF_ADD_CONF and self.bad==0:
                slots=[(self.anchor[ci],self.an[ci]),(self.tmpl[ci],self.tn[ci])]+list(zip(self.kf[ci],self.kfn[ci]))
                maxsim=max((fr*t).sum()/(fn*tn) for t,tn in slots)
                if maxsim < KF_THRESH:
                    self.kf[ci].append(fr); self.kfn[ci].append(fn)
                    if len(self.kf[ci])>K_KEYFRAMES:
                        # Evict the most REDUNDANT slot (highest similarity to some
                        # other kept slot), not simply the oldest — keeps distinct
                        # poses (front+side) instead of just whichever is newest.
                        kf,kn=self.kf[ci],self.kfn[ci]
                        worst_i=0; worst_s=-1.0
                        for i in range(len(kf)):
                            s=max(((kf[i]*kf[j]).sum()/(kn[i]*kn[j]) for j in range(len(kf)) if j!=i), default=-1.0)
                            if s>worst_s: worst_s=s; worst_i=i
                        kf.pop(worst_i); kn.pop(worst_i)
        frl=norm_patch(crop['y'],cx,cy,TMPL)
        self.tl=(1-TMPL_EMA)*self.tl+TMPL_EMA*frl; self.tln=nrm(self.tl)
        # Refresh the histogram cue only on a very clean lock (same anti-
        # contamination gate as keyframe banking) — an occlusion-tainted or
        # ambiguous frame must not corrupt the cumulative fg/bg model.
        if self.conf>=KF_ADD_CONF and self.bad==0:
            fg,bg = hist_counts_at(crop, cx, cy)
            self.histfg = (1-HIST_EMA)*self.histfg + HIST_EMA*fg
            self.histbg = (1-HIST_EMA)*self.histbg + HIST_EMA*bg

EGO = int(os.environ.get("EGO", 1))           # P1-A: ego-motion feed-forward on/off (A/B)
EGO_DEAD = 1.5                                # ignore sub-1.5px flow (noise, not a real pan)
EGO_CONS = 0.6                                # min grid-flow consensus to trust the ego estimate

FB_SEARCH = 4                                 # backward-match half-window (TLD-style FB check)
FB_MAX_ERR = 1.5                              # discard a grid point if the round trip exceeds this

def ego_estimate(prev, cur, patch=5, search=12, gx=8, gy=6, minvar=40, ex=None):
    """Mirror of OpticalFlow.kt: median grid-SSD displacement prev->cur (the
    ego/camera motion; the moving target is an outlier the median rejects).
    matchTemplate(TM_SQDIFF) gives the SSD surface per grid point fast.

    ex=(cx,cy,half) EXCLUDES grid points inside the current tracked box — if the
    target is a large fraction of the frame, its own motion can otherwise win the
    median vote (high "consensus") even though it isn't camera pan at all.

    Each surviving point is also checked FORWARD-BACKWARD (TLD-style): re-match
    the found position back toward its origin; a round trip that doesn't return
    close to the start means the match was ambiguous (aliased texture, not real
    motion) and is discarded before it can pollute the median/consensus.
    """
    import cv2
    H,W = prev.shape; m=patch+search
    if W<=2*m or H<=2*m: return 0.0,0.0,0.0
    dxs=[]; dys=[]; pxs=[]; pys=[]
    ego_estimate.last_pts=None
    for j in range(1,gy+1):
        for i in range(1,gx+1):
            cx=m+(W-2*m)*i//(gx+1); cy=m+(H-2*m)*j//(gy+1)
            if ex is not None:
                excx,excy,exhalf = ex
                if exhalf>0 and abs(cx-excx)<=exhalf and abs(cy-excy)<=exhalf: continue
            tp=prev[cy-patch:cy+patch+1, cx-patch:cx+patch+1]
            if tp.var()<minvar: continue
            reg=cur[cy-m:cy+m+1, cx-m:cx+m+1]
            res=cv2.matchTemplate(reg.astype(np.float32), tp.astype(np.float32), cv2.TM_SQDIFF)
            mn=np.unravel_index(np.argmin(res), res.shape)
            bdy=mn[0]-search; bdx=mn[1]-search
            # forward-backward check: match the found patch in `cur` back against `prev`
            fcx,fcy=cx+bdx,cy+bdy
            if fcx-m<0 or fcy-m<0 or fcx+m>=W or fcy+m>=H: continue
            bp=cur[fcy-patch:fcy+patch+1, fcx-patch:fcx+patch+1]
            breg=prev[cy-FB_SEARCH-patch:cy+FB_SEARCH+patch+1, cx-FB_SEARCH-patch:cx+FB_SEARCH+patch+1]
            if breg.shape[0]!=bp.shape[0]+2*FB_SEARCH or breg.shape[1]!=bp.shape[1]+2*FB_SEARCH: continue
            bres=cv2.matchTemplate(breg.astype(np.float32), bp.astype(np.float32), cv2.TM_SQDIFF)
            bmn=np.unravel_index(np.argmin(bres), bres.shape)
            fberr=np.hypot(bmn[0]-FB_SEARCH, bmn[1]-FB_SEARCH)
            if fberr>FB_MAX_ERR: continue          # forward match wasn't self-consistent — discard
            dxs.append(float(bdx)); dys.append(float(bdy)); pxs.append(float(cx)); pys.append(float(cy))
    if len(dxs)<4: return 0.0,0.0,0.0
    dxs=np.array(dxs); dys=np.array(dys); mdx=np.median(dxs); mdy=np.median(dys)
    ego_estimate.last_pts = (np.array(pxs,np.float32), np.array(pys,np.float32), dxs, dys)
    # consensus = fraction of grid points agreeing with the median (inliers). High
    # on a rigid camera pan; low under noise or a large independently-moving
    # occluder — so it, not the target's state, tells us when to trust the ego.
    cons=float(np.mean(np.hypot(dxs-mdx, dys-mdy) <= 2.0))
    return float(mdx), float(mdy), cons

# ---- STAPLE-style histogram appearance cue ---------------------------------
# Complements the spatial NCC cues: a chroma-histogram foreground/background
# score has NO spatial layout at all, so it survives deformation/rotation that
# breaks template correlation, at the cost of being weaker under illumination
# change (a known, accepted STAPLE trade-off). Fused into the SAME weighted-sum
# fusion as every other cue via its own PSR (peak-sharpness), not a fixed ratio
# — consistent with how every other cue here self-assesses its own weight.
HIST_BINS = 64            # 8x8 quantized (cu,cv)
HIST_HALF = TMPL / 2.0    # fg region = centred TMPL box (matches the anchor template)
HIST_BG_MARGIN = TMPL * 0.75   # buffer beyond fg excluded from bg (avoid boundary contamination)
HIST_LAMBDA = 1.0
HIST_EMA = 0.08
HIST_WEIGHT_CAP = float(os.environ.get('HIST_WEIGHT_CAP', 0.5))   # sim-swept sweet spot: 1.0 let
# histogram dominate over a merely-noisy (not truly occluded) spatial cue (noisy_occ/edge
# 98%->86% lock); 0.5 keeps most of the real occlusion win while fully recovering (even
# improving) the noisy case (edge 98%->100%, L+C 77%->89%).

def hist_bin_idx(crop):
    cu = np.clip(((crop['u'] + 128) / 32).astype(int), 0, 7)
    cv = np.clip(((crop['v'] + 128) / 32).astype(int), 0, 7)
    return cu * 8 + cv

def hist_counts_at(crop, cx, cy):
    """fg/bg per-bin pixel counts, fg = TMPL box centred at (cx,cy) — the ACTUAL
    found position, not the crop centre (which drifts from prediction error)."""
    H, W = crop['y'].shape
    yy, xx = np.mgrid[0:H, 0:W]
    dx = np.abs(xx - cx); dy = np.abs(yy - cy)
    fgm = (dx <= HIST_HALF) & (dy <= HIST_HALF)
    bgm = (dx > HIST_HALF + HIST_BG_MARGIN) | (dy > HIST_HALF + HIST_BG_MARGIN)
    bins = hist_bin_idx(crop)
    fg = np.bincount(bins[fgm], minlength=HIST_BINS).astype(np.float32)
    bg = np.bincount(bins[bgm], minlength=HIST_BINS).astype(np.float32)
    return fg, bg

def hist_response(crop, histfg, histbg, g0, g1, stride):
    """Per-candidate-position mean fg-probability, via an integral image so cost
    is O(crop + positions) instead of O(positions x template^2) like NCC."""
    bins = hist_bin_idx(crop)
    beta = histfg[bins] / (histfg[bins] + histbg[bins] + HIST_LAMBDA)
    ii = np.zeros((beta.shape[0] + 1, beta.shape[1] + 1), np.float32)
    ii[1:, 1:] = np.cumsum(np.cumsum(beta, axis=0), axis=1)
    gs = np.arange(g0, g1 + 1, stride); gw = len(gs); half = TMPL // 2
    H, W = beta.shape
    resp = np.zeros((gw, gw), np.float32)
    for gj, cy in enumerate(gs):
        for gi, cx in enumerate(gs):
            x0 = max(0, cx - half); y0 = max(0, cy - half)
            x1 = min(W, cx + half); y1 = min(H, cy + half)
            area = (x1 - x0) * (y1 - y0)
            if area <= 0: continue
            s = ii[y1, x1] - ii[y0, x1] - ii[y1, x0] + ii[y0, x0]
            resp[gj, gi] = s / area
    return resp

# ---- scenario generation (RGB -> y,u,v) ----
rng = np.random.RandomState(42)
BGT = rng.rand(240,320).astype(np.float32)*40+60   # textured background luma
# Wide background for PAN scenarios, from a SEPARATE RNG so adding it doesn't
# perturb the global rng state (which would shift every other scenario's noise).
BGT_WIDE = np.random.RandomState(7).rand(240,640).astype(np.float32)*40+60

def make_frame(H,W, tgt, extra=None, noise=6, occ=None, mark_ang=0.0, bg_off=None):
    if bg_off is None:
        y = resample(BGT,0,0,BGT.shape[1],BGT.shape[0],W,H).copy()
    else:                                    # PAN: sample a moving window of the wide bg
        y = resample(BGT_WIDE, bg_off[0], bg_off[1], W, H, W, H).copy()
    u = np.zeros((H,W),np.float32); v=np.zeros((H,W),np.float32)
    def stamp(cx,cy,rad,lum,cu,cv,ang=0.0):
        x0=int(cx-rad);x1=int(cx+rad);y0=int(cy-rad);y1=int(cy+rad)
        x0=max(0,x0);y0=max(0,y0);x1=min(W,x1);y1=min(H,y1)
        mx=cx+rad*0.4*np.cos(ang); my=cy+rad*0.4*np.sin(ang)   # off-centre mark, rotatable
        for yy in range(y0,y1):
            for xx in range(x0,x1):
                # Scale-STABLE structure: bright core + off-centre mark, sized as
                # a fraction of the target so its edges scale with it (like a real
                # object outline, not a fixed-frequency texture). The mark's ANGLE
                # models target rotation / pose change (appearance shift).
                d=np.hypot(xx-cx,yy-cy)/max(rad,1)
                t = 45 if d<0.45 else 0
                if abs(xx-mx)<rad*0.18 and abs(yy-my)<rad*0.18: t=-35
                y[yy,xx]=np.clip(lum+t,0,255); u[yy,xx]=cu; v[yy,xx]=cv
    cx,cy,rad,lum,cu,cv = tgt
    if extra: stamp(*extra)
    stamp(cx,cy,rad,lum,cu,cv,mark_ang)
    if occ:
        ox,oy,orad=occ
        x0=max(0,int(ox-orad));x1=min(W,int(ox+orad));y0=max(0,int(oy-orad));y1=min(H,int(oy+orad))
        y[y0:y1,x0:x1]=70; u[y0:y1,x0:x1]=0; v[y0:y1,x0:x1]=0
    y=np.clip(y+rng.randn(H,W)*noise,0,255).astype(np.float32)
    return {'y':y,'u':u.astype(np.float32),'v':v.astype(np.float32)}

def run(scenario, cues):
    frames, gt = scenario()
    tr=Tracker(cues)
    g0=gt[0]; tr.designate(frames[0], g0[0], g0[1], 44)
    errs=[]; locked=0; reacq=None; lost_since=None
    for i in range(1,len(frames)):
        bx,by,bs,conf,st,ax,ay=tr.update(frames[i])
        gx,gy=gt[i][0],gt[i][1]
        e=np.hypot(bx-gx,by-gy); errs.append(e)
        on = e<25
        if on: locked+=1
        # reacquire timing after occlusion window
    errs=np.array(errs)
    return dict(mean=errs.mean(), p90=np.percentile(errs,90), mx=errs.max(),
                lockpct=100*locked/len(errs))

# scenarios -----------------------------------------------------------------
def sc_translate():
    fr=[];gt=[]
    for i in range(40):
        cx=60+i*5.0; cy=120.0
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30)))
    return fr,gt
def sc_approach():
    fr=[];gt=[]
    for i in range(40):
        cx=80+i*4.0; cy=120.0; rad=12+i*0.7
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,rad,150,40,-30)))
    return fr,gt
def sc_occlusion():
    fr=[];gt=[]
    for i in range(40):
        cx=60+i*5.0; cy=120.0; occ=(cx,cy,26) if 15<=i<=21 else None
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30),occ=occ))
    return fr,gt
def sc_distractor():
    fr=[];gt=[]
    for i in range(40):
        cx=60+i*5.0; cy=120.0; dx=90+i*4.6  # similar object crossing near
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30),extra=(dx,135,14,150,40,-30)))
    return fr,gt
def sc_fast():
    fr=[];gt=[]
    for i in range(30):
        cx=40+i*9.0; cy=120.0
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30)))
    return fr,gt
def sc_noisy_occ():
    # High sensor noise (webcam-like) + a brief occlusion mid-run — the exact
    # setup that lets a spurious peak pump the velocity and send the box
    # "wandering off in random directions" out of frame. Runaway shows up as a
    # huge max error; the velocity cap + coast decay should bound it.
    fr=[];gt=[]
    for i in range(45):
        cx=70+i*4.0; cy=110.0+18*np.sin(i*0.4)          # gentle curve (not const-vel)
        occ=(cx,cy,24) if 18<=i<=24 else None
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30),occ=occ,noise=16))
    return fr,gt

def sc_rotate():
    # Target slowly drifts while its appearance ROTATES through a full turn — a
    # single EMA-adaptive template smears across poses and drops lock; a diverse
    # appearance bank should hold it by keeping distinct pose keyframes.
    fr=[];gt=[]
    for i in range(60):
        cx=90+i*1.5; cy=120.0; ang=i*0.12          # ~0.12 rad/frame → >1 full turn
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,16,150,40,-30),mark_ang=ang))
    return fr,gt
def sc_pan():
    # Fast camera PAN: the (nearly world-fixed) target sweeps across the frame with
    # the background at 8 px/frame — fast enough that constant-velocity prediction
    # lags and the target drifts toward the search-window edge. Ego-motion
    # feed-forward should recover the pan and keep the crop centred.
    fr=[];gt=[]
    for i in range(40):
        pan=8.0*i; cx=250-8.0*i+0.3*i; cy=120.0    # target rides the pan (+ slight own drift)
        gt.append((cx,cy))
        fr.append(make_frame(240,320,(cx,cy,15,150,40,-30),bg_off=(pan,0.0)))
    return fr,gt
def sc_recede():
    # Target flies AWAY: it shrinks and its contrast against the background falls,
    # so the response PSR legitimately declines while the lock stays perfectly
    # valid. Scale adaptation is exactly what this needs -- and exactly what the
    # occlusion detector freezes if its baseline can only ratchet upward.
    fr=[];gt=[]
    for i in range(55):
        cx=160.0+i*0.6; cy=120.0
        rad=max(7.0, 30.0-i*0.45)            # 30px -> 7px over the run
        lum=150-i*0.9                        # fades toward the ~80 background
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,rad,lum,30,-20)))
    return fr,gt
def sc_pan_large():
    # Same camera pan as sc_pan, but the target is LARGE relative to the FULL
    # FRAME (flow sampling runs on the whole incoming frame, not the crop — a
    # target must dominate the FRAME, not just the crop, to bias the grid median).
    # rad=90 in a 320x240 frame is a close-range/orbit-style shot (~56% of frame
    # width) — stresses whether the target's own motion corrupts the ego-motion
    # median (it shouldn't: box exclusion should keep the flow grid off it).
    fr=[];gt=[]
    for i in range(40):
        pan=6.0*i; cx=170-6.0*i+0.3*i; cy=120.0
        gt.append((cx,cy))
        fr.append(make_frame(240,320,(cx,cy,90,150,40,-30),bg_off=(pan,0.0)))
    return fr,gt
def sc_reacq():
    # Target is occluded for a stretch AND keeps moving fast behind the occluder,
    # so it REAPPEARS well outside the normal crop (bsize*2.2). Only the coasting
    # FOV zoom-out can re-find it. Without it, the box coasts off and never re-locks.
    fr=[];gt=[]
    for i in range(45):
        cx=60+i*7.0; cy=120.0
        occ=(60+16*7.0, cy, 30) if 14<=i<=26 else None   # occluder fixed; target drives on
        gt.append((cx,cy)); fr.append(make_frame(240,320,(cx,cy,14,150,40,-30),occ=occ))
    return fr,gt

SCEN = dict(translate=sc_translate, approach=sc_approach, occlusion=sc_occlusion,
            distractor=sc_distractor, fast=sc_fast, noisy_occ=sc_noisy_occ, reacq=sc_reacq,
            rotate=sc_rotate, pan=sc_pan, pan_large=sc_pan_large, recede=sc_recede)
CUESETS = {'none':['none'], 'edge':['edge'],
           'FUSE3':['edge','chroma','none'],   # incl. edge (scale-fragile)
           'L+C':['none','chroma'],            # luma+chroma, both scale-robust
           # --- hue-preserving variants (see apply_cue) ---
           'HUE':   ['edge','cu','cv','none'],       # chroma magnitude -> U,V
           'HUE2':  ['none','cu','cv'],              # drop edge, keep it cheap
           'HUE4':  ['none','cu','cv','co1','co2'],  # + 45-deg opponent pair
           'HUEmag':['edge','chroma','cu','cv','none']}  # magnitude AND hue

def main():
    print(f"{'scenario':<11}{'cues':<7}{'mean':>7}{'p90':>7}{'max':>7}{'lock%':>7}")
    for sname,sfn in SCEN.items():
        for cname,cues in CUESETS.items():
            r=run(sfn,cues)
            print(f"{sname:<11}{cname:<7}{r['mean']:7.1f}{r['p90']:7.1f}{r['mx']:7.1f}{r['lockpct']:7.0f}")
        print()

if __name__ == '__main__':      # importable (eval_tracker reuses Tracker/SCEN/CUESETS)
    main()


def ego_affine_at(tx, ty):
    """Ego displacement AT A GIVEN POINT, from a similarity fit to the grid
    correspondences ego_estimate() already computed.

    ego_estimate returns a MEDIAN TRANSLATION -- one vector for the whole frame.
    That is only the true camera motion if the camera is panning; a drone that is
    closing on a target also zooms and rolls, and under zoom the image
    displacement is proportional to distance from the focus of expansion, so a
    single vector is wrong everywhere except at one radius.

    The correspondences needed to do better are already computed and discarded.
    Fitting a similarity (scale + rotation + translation) over the inliers costs
    a 4-parameter least squares on ~40 points and gives the displacement at the
    TARGET's position specifically.

    Returns (dx, dy) at (tx,ty), or None when there is not enough support.
    """
    pts = getattr(ego_estimate, 'last_pts', None)
    if pts is None:
        return None
    px, py, dx, dy = pts
    if len(px) < 6:
        return None
    # NOT filtered by agreement with the median translation. That was the first
    # implementation and it self-tested to "no answer" on a synthetic zoom: under
    # zoom the displacements legitimately DISAGREE with any single vector, so a
    # median-translation inlier test discards precisely the points carrying the
    # information this function exists to recover. Fit first, then reject by
    # residual TO THE FITTED MODEL, then refit.
    # similarity: [x'] = [a -b][x] + [c]   -- 4 unknowns, robust with few points.
    #             [y']   [b  a][y]   [d]     A full affine (6) overfits 40 noisy
    #                                        grid matches and can shear wildly.
    def fit(px, py, dx, dy):
        X, Y = px + dx, py + dy
        A = np.zeros((2 * len(px), 4), np.float64)
        b = np.zeros(2 * len(px), np.float64)
        A[0::2, 0] = px; A[0::2, 1] = -py; A[0::2, 2] = 1.0
        A[1::2, 0] = py; A[1::2, 1] = px;  A[1::2, 3] = 1.0
        b[0::2] = X; b[1::2] = Y
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        return sol

    try:
        sol = fit(px, py, dx, dy)
        a_, b_, c_, d_ = sol
        rx = (a_ * px - b_ * py + c_) - (px + dx)
        ry = (b_ * px + a_ * py + d_) - (py + dy)
        keep = np.hypot(rx, ry) <= 3.0
        if keep.sum() >= 6 and keep.sum() < len(px):
            sol = fit(px[keep], py[keep], dx[keep], dy[keep])
        a_, b_, c_, d_ = sol
    except np.linalg.LinAlgError:
        return None
    nx = a_ * tx - b_ * ty + c_
    ny = b_ * tx + a_ * ty + d_
    return float(nx - tx), float(ny - ty)
