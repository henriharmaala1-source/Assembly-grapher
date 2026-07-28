#!/usr/bin/env python3
import os, sys, numpy as np, cv2
sys.path.insert(0,'.')
import eval_tracker as et, eval_vs_learned as ev
def csrt(**kw):
    def f():
        q=cv2.TrackerCSRT_Params()
        for k,v in kw.items(): setattr(q,k,v)
        return cv2.TrackerCSRT_create(q)
    return f
B=dict(use_segmentation=False)          # segmentation already shown worthless
CFG=[('full(no-seg)',    csrt(**B)),
     ('-channel wts',    csrt(**B, use_channel_weights=False)),
     ('-HOG (CN+gray)',  csrt(**B, use_hog=False)),
     ('-CN  (HOG+gray)', csrt(**B, use_color_names=False)),
     ('-gray(HOG+CN)',   csrt(**B, use_gray=False)),
     ('1 scale',         csrt(**B, number_of_scales=1))]
names=sorted(n[:-4] for n in os.listdir('clips') if n.endswith('.mp4'))
acc={k:[] for k,_ in CFG}; ms={k:[] for k,_ in CFG}
for nm in names:
    fr=et.read_video(f'clips/{nm}.mp4'); gt,dz=et.read_labels(f'clips/{nm}.csv',len(fr))
    line=f"{nm:<21}"
    for k,mk in CFG:
        try:
            r=ev.run_learned(fr,gt,dz,25.0,mk); a=r.get('on_pct',0); t=r['ms']
        except Exception: a,t=float('nan'),float('nan')
        acc[k].append(a); ms[k].append(t); line+=f"{a:>7.0f}%"
    print(line, flush=True)
print('-'*70)
print(f"{'MEAN':<21}"+"".join(f"{np.nanmean(acc[k]):>7.0f}%" for k,_ in CFG))
print(f"{'ms/frame':<21}"+"".join(f"{np.nanmean(ms[k]):>8.1f}" for k,_ in CFG))
