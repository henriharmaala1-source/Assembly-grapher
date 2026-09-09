#!/usr/bin/env python3
"""
Wide-DCF re-detection: can a cheap, wide correlation filter rescue the
classical tracker when it drops the lock?

Cost says yes. Over the same area LockTracker's wide re-acquire evaluates 1,600
positions for 3.8M ops; a 192x192 DCF evaluates 36,864 for 2.5M -- denser AND
cheaper, because FFT search grows as N log N rather than positions x
template^2. Self-tested, the wide filter doubles the recovery range (+-40px to
+-80px) and its PSR separates found (9-11) from lost (3.0-3.6).

Accuracy says no. Three variants, three negatives:

    NCC   always rebuild   always relocate   verdict .30   .45   .60
    70%        69%              70%              67%       67%   67%

The failure is the HANDOVER, not the re-detection. Rebuilding the templates
(designate) takes a fresh appearance model from the frame where tracking just
failed -- fine when the target genuinely changed (d_pan_shake 71 -> 95), fatal
when a distractor is in the box (h_clutter 99 -> 77). Relocating keeps the good
model and inverts both results exactly (69 / 100).

The third attempt supplied the missing verdict: NCC against the FROZEN anchors,
the one thing that cannot have drifted, to ask whether the target still looks
like the designation. It came out backwards on both clips and identical at every
threshold, because low anchor similarity means either "the target changed" or
"this is not the target" -- two situations needing OPPOSITE actions that one
patch of evidence cannot separate.

Kept so the result is reproducible and so the variants can be re-run on REAL
footage, where the balance may differ: if analog clips look like d_pan_shake
(brief drop, appearance intact) then always-rebuild is simply right for this
domain and the decision disappears.

    python3 eval_redetect.py
"""
sys.path.insert(0,'.')
import eval_tracker as et, eval_vs_learned as ev, simtrack as st
from dcf_tracker import DCFTracker
cues=st.CUESETS.get('FUSE3',['edge','chroma','none'])

def anchor_sim(ncc, frame, cx, cy, size):
    """How much does the patch at (cx,cy) still look like the ORIGINAL
    designated view? NCC against the fixed anchors, best cue.

    The anchors are the one thing in the tracker that cannot have drifted --
    they are the designation, frozen. So this is a verdict on the TARGET's
    appearance, not on the tracker's current belief about it."""
    crop = st.crop_raw(frame, cx, cy, size)
    best = -2.0
    for ci, c in enumerate(ncc.cues):
        p = st.norm_patch(st.apply_cue(crop, c), st.CROP/2, st.CROP/2, st.TMPL)
        s = float(np.dot(p.ravel(), ncc.anchor[ci].ravel()) / (st.nrm(p)*ncc.an[ci]))
        best = max(best, s)
    return best

def redetect(fr,gt,dz,mode='verdict',thresh=0.45,patience=2,psr_accept=6.0,refresh=5,padding=6.0):
    n=len(fr); fi0,cx0,cy0,sz0=dz
    yuv=[et.bgr_to_yuvdict(f) for f in fr]
    ncc=st.Tracker(cues); ncc.designate(yuv[fi0],cx0,cy0,sz0)
    wide=DCFTracker(channels=('y',)).make_wide(yuv[fi0],cx0,cy0,sz0,padding=padding)
    errs=np.full(n,np.nan,np.float32); states=['IDLE']*n
    bad=0; nre=0; nrel=0
    for i in range(fi0+1,n):
        bx,by,bs,conf,state,_,_=ncc.update(yuv[i])
        bad = 0 if state=='LOCKED' else bad+1
        if bad<=patience:
            cx,cy=bx,by; states[i]=state
            if state=='LOCKED' and (i%refresh)==0: wide.init(yuv[i],bx,by,bs)
        else:
            wide.cx,wide.cy=bx,by; wide.update(yuv[i])
            if wide.score>=psr_accept:
                cx,cy=wide.cx,wide.cy; states[i]='LOCKED'; bad=0
                if mode=='designate': rebuild=True
                elif mode=='relocate': rebuild=False
                else:
                    # LOW similarity to the frozen anchors => the target itself
                    # has changed, so the old model is stale and rebuilding is
                    # right. HIGH similarity => the model is still good and the
                    # failure was positional, so rebuilding would only risk
                    # swallowing whatever else is in the box.
                    rebuild = anchor_sim(ncc, yuv[i], cx, cy, bs) < thresh
                if rebuild:
                    ncc.designate(yuv[i],cx,cy,bs); nre+=1
                else:
                    ncc.bcx=cx; ncc.bcy=cy; ncc.x=cx; ncc.y=cy
                    ncc.vx=0.0; ncc.vy=0.0; ncc.bad=0; ncc.state='LOCKED'; nrel+=1
            else:
                cx,cy=bx,by; states[i]=state
        if gt is not None and gt[i] is not None:
            gx,gy,_=gt[i]; errs[i]=float(np.hypot(cx-gx,cy-gy))
    m=et.metrics(errs,states,25.0,gt is not None); m['re']=nre; m['rel']=nrel; return m

CFG=[('always rebuild',dict(mode='designate')),('always relocate',dict(mode='relocate')),
     ('verdict .30',dict(mode='verdict',thresh=0.30)),('verdict .45',dict(mode='verdict',thresh=0.45)),
     ('verdict .60',dict(mode='verdict',thresh=0.60))]
names=sorted(n[:-4] for n in os.listdir('clips') if n.endswith('.mp4'))
base=[]; K={k:[] for k,_ in CFG}
print(f"{'clip':<21}{'NCC':>6}"+"".join(f"{k:>16}" for k,_ in CFG), flush=True)
for nm in names:
    f=et.read_video(f'clips/{nm}.mp4'); gt,dz=et.read_labels(f'clips/{nm}.csv',len(f))
    a=ev.run_classical(f,gt,dz,cues,25.0).get('on_pct',0); base.append(a)
    line=f"{nm:<21}{a:>5.0f}%"
    for k,kw in CFG:
        m=redetect(f,gt,dz,**kw); K[k].append(m.get('on_pct',0)); line+=f"{m.get('on_pct',0):>15.0f}%"
    print(line, flush=True)
print('-'*88)
print(f"{'MEAN':<21}{np.mean(base):>5.0f}%"+"".join(f"{np.mean(K[k]):>15.0f}%" for k,_ in CFG))
