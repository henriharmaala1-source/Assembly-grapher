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
PSR_LOCK, PSR_WARN = 5.5, 3.8
SIZE_FLOOR = 36.0                             # matches LockTracker.kt (anti over-zoom)
OCC_FRAC = 0.55                               # P2-B: PSR below this x clean-baseline = occluded
OCC_ENTER = 2                                 # consecutive low frames before declaring occlusion
OCC_MAX = 20                                  # after this many, re-baseline (not an occluder)
EARLY_TERM_PSR = 10.0                         # skip remaining cues once one is this dominant

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
    gs = np.arange(g0, g1+1, stride); gw = len(gs)
    resp = np.full((gw,gw), -2.0, np.float32); h = TMPL//2
    for gj,cy in enumerate(gs):
        for gi,cx in enumerate(gs):
            x0=cx-h; y0=cy-h
            if x0<0 or y0<0 or x0+TMPL>chan.shape[1] or y0+TMPL>chan.shape[0]: continue
            p = chan[y0:y0+TMPL, x0:x0+TMPL]; p = p-p.mean()
            resp[gj,gi] = (p*tmpl).sum()/(tn*nrm(p))
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
    def designate(self, frame, px, py, size):
        self.bcx=px; self.bcy=py; self.bsize=float(np.clip(size,SIZE_FLOOR,min(frame['y'].shape)))
        crop=crop_raw(frame,px,py,self.bsize)
        self._build(crop)
        self.x=px; self.y=py; self.vx=0.0; self.vy=0.0
        self.bad=0; self.conf=1; self.state='LOCKED'; self.psrema=0.0; self.prevY=frame['y']; self.occluded=False; self.occlow=0
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
            edx=float(np.clip(fx,-cap,cap)); edy=float(np.clip(fy,-cap,cap))
        self.prevY=frame['y']
        # predict (alpha-beta + ego)
        self.x+=self.vx+edx; self.y+=self.vy+edy; pcx,pcy=self.x,self.y
        # zoom the search out (see LockTracker.kt) — only AFTER normal coasting has
        # failed for FOV_DELAY frames (early coasting rides the const-vel prediction
        # onto the target; zooming out early hurts). Coarser stride holds cost flat.
        wide = self.bad >= FOV_DELAY
        fov = min(1+0.3*(self.bad-FOV_DELAY+1), 3.0) if wide else 1.0
        croppix = (int(CROP*fov)//2)*2 if fov>1 else CROP
        stride_eff = max(1, int(STRIDE*fov))
        regionw = self.bsize*MARGIN*fov
        crop = {k: resample(frame[k], pcx-regionw/2, pcy-regionw/2, regionw, regionw, croppix, croppix)
                for k in ('y','u','v')}
        speed=np.hypot(self.vx,self.vy); velcrop=speed*CROP/(self.bsize*MARGIN)
        maxhalf=croppix//2-TMPL//2
        half = maxhalf if self.bad>0 else int(np.clip(SEARCH+velcrop*2,SEARCH,maxhalf))
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
        else:
            self.vx*=0.6; self.vy*=0.6          # coast decelerates instead of flying off
            self.bcx,self.bcy=pcx,pcy; self.bad+=1
            self.state=('LOST' if self.bad>=LOSS_TIMEOUT else
                        'SEARCHING' if wide else 'COASTING')
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
    dxs=[]; dys=[]
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
            dxs.append(float(bdx)); dys.append(float(bdy))
    if len(dxs)<4: return 0.0,0.0,0.0
    dxs=np.array(dxs); dys=np.array(dys); mdx=np.median(dxs); mdy=np.median(dys)
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
           'L+C':['none','chroma']}            # luma+chroma, both scale-robust

def main():
    print(f"{'scenario':<11}{'cues':<7}{'mean':>7}{'p90':>7}{'max':>7}{'lock%':>7}")
    for sname,sfn in SCEN.items():
        for cname,cues in CUESETS.items():
            r=run(sfn,cues)
            print(f"{sname:<11}{cname:<7}{r['mean']:7.1f}{r['p90']:7.1f}{r['mx']:7.1f}{r['lockpct']:7.0f}")
        print()

if __name__ == '__main__':      # importable (eval_tracker reuses Tracker/SCEN/CUESETS)
    main()
