#!/usr/bin/env python3
"""
Which part of CSRT is actually worth having? Ablation and cost, not intuition.

VALUE -- mean on-target over the 10-clip battery (segmentation off in every
column, since turning it off changed 88% -> 88%):

                        noseg  -chwt  -HOG   -CN    -gray  1scale
    f_maneuver           100%   100%   100%    54%    100%   100%
    z_below_floor         80%    79%    78%    21%     80%    79%
    g_occlusion           31%    31%    31%    31%     34%    31%
    MEAN                  88%    89%    87%    77%     88%    87%

    segmentation      0 points   the ADMM spatial reliability map
    channel weights  +0 points   89% WITHOUT them
    33-scale search  +1 point
    HOG              +1 point
    Colour Names    +11 points   the entire advantage

COST -- measured separately on an idle machine (d_pan_shake, 50 frames):

    component                    ms      value      ms per point
    HOG                       115.9      +1 pt          116
    Colour Names               69.1     +11 pts           6.3
    channel reliability wts    33.1       0 pts          inf
    33-scale DSST              19.9      +1 pt            20
    segmentation (ADMM mask)    9.1       0 pts          inf
    greyscale                   6.3       0 pts          inf
    full CSRT                 241.4      88%

HOG is 48% of the entire runtime and buys ONE point -- the worst trade in the
algorithm. Colour Names is the only component that pays for itself, and it is
the one that did NOT reproduce outside CSRT (hue as NCC cues: 70% -> 55-60%;
colour channels inside the DCF: 78% -> 76%).

The segmentation is NOT expensive: 9 ms. It is the most intricate part of the
paper and nearly free -- it simply does nothing measurable here. So "skip the
segmentation" is a VALUE argument, not a cost one.

Where Colour Names carries it is diagnostic: f_maneuver 100 -> 54 and
z_below_floor 80 -> 21 without it, both blur/contrast cases. Motion blur
averages neighbouring pixels, destroying gradient and texture structure -- HOG,
edges and NCC template matching degrade together -- while barely touching mean
colour. Colour is blur-robust; texture is not.

THREE THINGS FROM THE SOURCE THAT ARE NOT PARAMETERS
----------------------------------------------------
Found by reading trackerCSRT.cpp; invisible to any parameter sweep.

1. THE BOX IS NEVER ACCUMULATED.
       bounding_box.width = current_scale_factor * original_target_size.width;
   Size stays anchored to the DESIGNATION and is multiplied by one scalar,
   itself clamped to [min_scale_factor, max_scale_factor]. No frame-to-frame
   feedback, so the box cannot run away by construction. That is precisely the
   defect the ONNX tracker has -- it accumulates -- and this is a better fix
   than the 0.25-4x bound added there, because it removes the loop instead of
   fencing it. Costs nothing.

2. ADAPTIVE CELL SIZE.
       cell_size = clamp(ceil(w*h / 400), 1, 4)
   Small targets get full-resolution cells, large ones coarser. Filter
   resolution follows the target rather than being fixed -- which matters at
   long range, where a fixed-size filter spends most of its capacity on
   background. Costs nothing.

3. LOST DETECTION ON THE RAW RESPONSE.
       if (max_val < params.psr_threshold) return Point2f(-1,-1);
   A sub-threshold peak returns NO position rather than a guess. Cheap, and it
   is the honest-failure signal a gate needs -- the thing the ONNX tracker's
   score fails to give on real footage, where it never drops.

    PYTHONPATH=./ocv python3 ablate_csrt.py
"""
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
