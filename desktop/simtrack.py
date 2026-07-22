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
PSR_LOCK, PSR_WARN = 5.5, 3.8
SIZE_FLOOR = 36.0                             # matches LockTracker.kt (anti over-zoom)

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
        self.bad=0; self.conf=1; self.state='LOCKED'
    def _build(self, crop):
        self.tmpl=[]; self.tn=[]
        for c in self.cues:
            t=norm_patch(apply_cue(crop,c), CROP/2, CROP/2, TMPL)
            self.tmpl.append(t); self.tn.append(nrm(t))
        # fixed ANCHORS = the original views (anti-drift; used alone in wide search).
        self.anchor=[t.copy() for t in self.tmpl]; self.an=list(self.tn)
        # FIX 1: dedicated LUMA template for scale (edge channel is unreliable
        # for scale — approach scenario showed 22px vs 1.7px).
        self.tl=norm_patch(crop['y'], CROP/2, CROP/2, TMPL); self.tln=nrm(self.tl)
    def update(self, frame):
        # predict (alpha-beta)
        self.x+=self.vx; self.y+=self.vy; pcx,pcy=self.x,self.y
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
            # anchor alone during a wide re-acquire; anchor-OR-adaptive otherwise.
            rb,_=ncc_map(chan,self.anchor[ci],self.an[ci],g0,g1,stride_eff)
            if wide: r=rb
            else:
                ra,_=ncc_map(chan,self.tmpl[ci],self.tn[ci],g0,g1,stride_eff)
                r=np.maximum(ra,rb)
            gw=r.shape[0]; cc=(gw-1)/2
            if sig_p is None: sig_p = gw/1.4 if self.bad>0 else gw/2.5
            pk=np.unravel_index(np.argmax(r),r.shape)
            # (a) prediction-proximity: a cue whose peak drifts off (confidently-
            #     wrong edge under scale, or a distractor lock) is down-weighted,
            #     so the cue on the predicted target wins. PSR alone can't — a
            #     sharp-but-wrong peak has high PSR.
            prox=np.exp(-(((pk[1]-cc)**2+(pk[0]-cc)**2)/(2*sig_p*sig_p)))
            w=max(0,psr_of(r)-3)*prox
            if w<=0: continue
            anyw+=w; fused=r*w if fused is None else fused+r*w
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
            self.conf=psr2conf(psr_of(fused))
        else: self.conf=0
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
            self._scale(crop,cxc,cyc)
            if self.conf>=psr2conf(PSR_LOCK): self._adapt(crop,cxc,cyc)
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
            fr=norm_patch(apply_cue(crop,c),cx,cy,TMPL)
            self.tmpl[ci]=(1-TMPL_EMA)*self.tmpl[ci]+TMPL_EMA*fr; self.tn[ci]=nrm(self.tmpl[ci])
        frl=norm_patch(crop['y'],cx,cy,TMPL)
        self.tl=(1-TMPL_EMA)*self.tl+TMPL_EMA*frl; self.tln=nrm(self.tl)

# ---- scenario generation (RGB -> y,u,v) ----
rng = np.random.RandomState(42)
BGT = rng.rand(240,320).astype(np.float32)*40+60   # textured background luma

def make_frame(H,W, tgt, extra=None, noise=6, occ=None):
    y = resample(BGT,0,0,BGT.shape[1],BGT.shape[0],W,H).copy()
    u = np.zeros((H,W),np.float32); v=np.zeros((H,W),np.float32)
    def stamp(cx,cy,rad,lum,cu,cv,tex=True):
        x0=int(cx-rad);x1=int(cx+rad);y0=int(cy-rad);y1=int(cy+rad)
        x0=max(0,x0);y0=max(0,y0);x1=min(W,x1);y1=min(H,y1)
        for yy in range(y0,y1):
            for xx in range(x0,x1):
                # Scale-STABLE structure: bright core + off-centre mark, sized as
                # a fraction of the target so its edges scale with it (like a real
                # object outline, not a fixed-frequency texture).
                d=np.hypot(xx-cx,yy-cy)/max(rad,1)
                t = 45 if d<0.45 else 0
                if abs(xx-(cx+rad*0.4))<rad*0.18 and abs(yy-cy)<rad*0.18: t=-35
                y[yy,xx]=np.clip(lum+t,0,255); u[yy,xx]=cu; v[yy,xx]=cv
    cx,cy,rad,lum,cu,cv = tgt
    if extra: stamp(*extra)
    stamp(cx,cy,rad,lum,cu,cv)
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
            distractor=sc_distractor, fast=sc_fast, noisy_occ=sc_noisy_occ, reacq=sc_reacq)
CUESETS = {'none':['none'], 'edge':['edge'],
           'FUSE3':['edge','chroma','none'],   # incl. edge (scale-fragile)
           'L+C':['none','chroma']}            # luma+chroma, both scale-robust

print(f"{'scenario':<11}{'cues':<7}{'mean':>7}{'p90':>7}{'max':>7}{'lock%':>7}")
for sname,sfn in SCEN.items():
    for cname,cues in CUESETS.items():
        r=run(sfn,cues)
        print(f"{sname:<11}{cname:<7}{r['mean']:7.1f}{r['p90']:7.1f}{r['mx']:7.1f}{r['lockpct']:7.0f}")
    print()
