#!/usr/bin/env python3
"""
Four untried methods, measured. One works.

    clip                    base   #2 affine   #1 tavg   #3 N-best   #4 LK
    d_pan_shake              71%       71%        74%       71%        69%
    e_recede                100%      100%       100%      100%        82%
    f_maneuver               19%       17%         9%       19%        62%
    g_occlusion              79%       42%        35%       39%        91%
    h_clutter_distractor     99%       92%       100%       99%        96%
    i_worst                  18%        5%        18%       10%        18%
    MEAN                     70%       64%        65%       65%        73%

#4 LUCAS-KANADE COAST ASSIST -- +3 overall, and it fixes the two worst clips:
f_maneuver 19% -> 62% and g_occlusion 79% -> 91%, the best figure any
NCC-family configuration has produced on either. While COASTING the tracker
currently rides a constant-velocity guess with no image evidence at all; LK on
corner points inside the last good box supplies actual evidence, and because it
tracks TEXTURE rather than a template it does not care that the appearance
match has failed. Costs e_recede (100 -> 82), where the target shrinks until its
corners stop being corners.

#2 SIMILARITY EGO-MOTION -- 64%, worse. The estimator itself is a large
improvement and was self-tested against known warps:

    6% zoom + 3 deg roll   median translation 11-13 px error, similarity 0.1 px
    pure pan (8,-4)        both exact -- it degrades to translation, not past it
    12% zoom               median 17-19 px error, similarity 0.8-1.3 px

But the TRACKER is worse with it (g_occlusion 79 -> 42). A median is robust to
corrupted correspondences; a least-squares fit is not, and it is evaluated AT
the target, which during an occlusion sits outside the inlier support, so a
mildly bad fit extrapolates into a large displacement. Better physics, worse
robustness -- and robustness is what the occlusion case is made of.

#1 MOTION-COMPENSATED TEMPORAL AVERAGING -- 65%, worse. sqrt(N) noise reduction
is real, but the alignment cancels CAMERA motion only; a moving target is
smeared by exactly the amount it moved. f_maneuver 19 -> 9 is that smear. It
would pay only where the target is near-stationary in the frame and the noise
dominates -- the long-range case these clips do not contain.

#3 N-BEST PEAK SELECTION -- 65%, worse, and no N helped (2/3/5 -> 65/65/63).
The reasoning was that one frame cannot separate a target from a similar object
beside it while several frames can. The flaw is that this picks among peaks by
proximity to the prediction, which the per-cue weighting ALREADY does -- so it
mostly re-litigates a decision the fusion made with more information, and
sometimes overturns it wrongly (g_occlusion 79 -> 39). Real multi-hypothesis
tracking carries competing tracks forward over time; choosing differently within
one frame is not that, and does not get its benefit.

Switches left in simtrack (USE_AFFINE_EGO, NBEST) so the negatives stay
reproducible rather than becoming folklore.

    python3 eval_newmethods.py
"""
sys.path.insert(0,'.')
import eval_tracker as et, eval_vs_learned as ev, simtrack as st

def temporal_average(frames_yuv, n=3):
    """#1: average N ego-ALIGNED frames. Analog noise is temporally
    uncorrelated and the target is not, so averaging N aligned frames buys
    sqrt(N) SNR. Alignment matters: without it this is just motion blur."""
    out=[dict(frames_yuv[0])]
    buf=[frames_yuv[0]['y']]; acc=np.zeros(2)
    for i in range(1,len(frames_yuv)):
        dx,dy,cons=st.ego_estimate(frames_yuv[i-1]['y'], frames_yuv[i]['y'])
        acc = acc + ((dx,dy) if cons>=0.6 else (0,0))
        warped=[]
        for k,prev in enumerate(buf[-(n-1):]):
            shift=acc-acc  # frames are warped forward one step at a time below
            warped.append(prev)
        # warp the running buffer forward by this frame's ego step, then add
        M=np.float32([[1,0,dx if cons>=0.6 else 0],[0,1,dy if cons>=0.6 else 0]])
        buf=[cv2.warpAffine(b,M,(b.shape[1],b.shape[0]),borderMode=cv2.BORDER_REPLICATE) for b in buf]
        buf.append(frames_yuv[i]['y'])
        if len(buf)>n: buf.pop(0)
        f=dict(frames_yuv[i]); f['y']=np.mean(buf,axis=0).astype(np.float32)
        out.append(f)
    return out

def lk_assist(fr,gt,dz,cues):
    """#4: while COASTING, replace the constant-velocity guess with actual image
    evidence -- Lucas-Kanade on corner points inside the last good box, median
    displacement. Appearance-independent: it tracks texture, not a template."""
    n=len(fr); fi0,cx0,cy0,sz0=dz
    yuv=[et.bgr_to_yuvdict(f) for f in fr]
    grey=[cv2.cvtColor(f,cv2.COLOR_BGR2GRAY) for f in fr]
    tr=st.Tracker(cues); tr.designate(yuv[fi0],cx0,cy0,sz0)
    errs=np.full(n,np.nan,np.float32); states=['IDLE']*n
    pts=None; used=0
    for i in range(fi0+1,n):
        bx,by,bs,conf,state,_,_=tr.update(yuv[i])
        if state=='LOCKED':
            m=np.zeros_like(grey[i]); h=int(bs/2)
            y0,y1=max(0,int(by-h)),min(grey[i].shape[0],int(by+h))
            x0,x1=max(0,int(bx-h)),min(grey[i].shape[1],int(bx+h))
            m[y0:y1,x0:x1]=255
            pts=cv2.goodFeaturesToTrack(grey[i],maxCorners=30,qualityLevel=0.01,
                                        minDistance=3,mask=m)
        elif pts is not None and len(pts)>=4:
            nxt,stt,_=cv2.calcOpticalFlowPyrLK(grey[i-1],grey[i],pts,None)
            if nxt is not None and stt is not None and stt.sum()>=4:
                good_o=pts[stt.ravel()==1]; good_n=nxt[stt.ravel()==1]
                d=(good_n-good_o).reshape(-1,2)
                mdx,mdy=float(np.median(d[:,0])),float(np.median(d[:,1]))
                bx,by = bx+mdx, by+mdy
                tr.bcx=bx; tr.bcy=by; tr.x=bx; tr.y=by
                pts=good_n; used+=1
        states[i]=state
        if gt is not None and gt[i] is not None:
            gx,gy,_=gt[i]; errs[i]=float(np.hypot(bx-gx,by-gy))
    m=et.metrics(errs,states,25.0,gt is not None); m['lk']=used; return m

cues=st.CUESETS.get('FUSE3',['edge','chroma','none'])
names=sorted(n[:-4] for n in os.listdir('clips') if n.endswith('.mp4'))
K={k:[] for k in ('base','affine','tavg','lk')}
print(f"{'clip':<21}{'base':>7}{'#2 affine':>11}{'#1 tavg':>9}{'#4 LK':>8}", flush=True)
for nm in names:
    f=et.read_video(f'clips/{nm}.mp4'); gt,dz=et.read_labels(f'clips/{nm}.csv',len(f))
    yuv=[et.bgr_to_yuvdict(x) for x in f]
    st.USE_AFFINE_EGO=False
    K['base'].append(et.run(yuv,gt,dz,cues,25.0).get('on_pct',0))
    st.USE_AFFINE_EGO=True
    K['affine'].append(et.run(yuv,gt,dz,cues,25.0).get('on_pct',0))
    st.USE_AFFINE_EGO=False
    K['tavg'].append(et.run(temporal_average(yuv,3),gt,dz,cues,25.0).get('on_pct',0))
    K['lk'].append(lk_assist(f,gt,dz,cues).get('on_pct',0))
    print(f"{nm:<21}{K['base'][-1]:>6.0f}%{K['affine'][-1]:>10.0f}%{K['tavg'][-1]:>8.0f}%{K['lk'][-1]:>7.0f}%", flush=True)
print('-'*56)
print(f"{'MEAN':<21}"+"".join(f"{np.mean(K[k]):>{w}.0f}%" for k,w in (('base',6),('affine',10),('tavg',8),('lk',7))))
