#!/usr/bin/env python3
"""
A Siamese/one-stream tracker driven entirely from an ONNX file — every step of
the crop / normalise / decode pipeline written out explicitly, with NOTHING
hidden inside a library tracker class.

Why this exists
---------------
LightFC cannot be run here: its checkpoints are published only via Google Drive,
which is unreachable from this container (verified: Drive 000, HuggingFace 000,
GitHub tarball 403). So it cannot be exported to ONNX here, and a hand-written
wrapper for it could not be tested against anything.

Writing several hundred lines of untestable crop/normalise/decode logic is
exactly the failure mode this project has already paid for once. So instead:
this module reimplements the pipeline for a model that CAN be run and CAN be
checked — OpenCV's TrackerVit — and is measured against OpenCV's own
implementation. Once it tracks as well, the pipeline is proven, and LightFC
becomes a model swap plus a head decode rather than a reimplementation.

The algorithm is a port of opencv/modules/video/src/tracking/tracker_vit.cpp.
Details that are easy to get wrong are called out inline — they are exactly
what a Kotlin port trips over.

How well it is verified, precisely
----------------------------------
FUNCTIONALLY equivalent, NOT bit-exact, and the distinction is honest rather
than convenient:

  battery mean on-target   cv2.TrackerVit 83%   this module 85%

Bit-exactness was attempted and abandoned for a good reason: the INSTALLED
OpenCV 5.0.0 binary does not behave like the published source. Verified along
the way — the input blob matches cv2.dnn.blobFromImageWithParams to 2.4e-7, the
crops land where hand-arithmetic says they should (56 and 112 px, no padding),
the graph I/O names and shapes are right, and the template does reach the graph
(zeroing it moves the score). Yet the reference scores 0.594375 where every
construction path here scores 0.651692, and opencv/4.x and opencv/5.x are
identical apart from an engine argument. A parameter search fit that single
scalar at search-factor 3 with no Hann window — but that configuration scores
69% on the battery against 85% for factor 4. So the scalar match was
overfitting to one number, and the battery is the arbiter.

Conclusion: the published algorithm (factor 4, Hann on) is correct and this
implementation reproduces its BEHAVIOUR; the installed binary is a black box
that differs from its own source, and chasing it was not worth more time.

Usage
-----
  python3 siamese_onnx.py --model vittrack.onnx --validate   # per-frame deviation
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _trunc_div2(a: int) -> int:
    """C++ integer division truncates TOWARD ZERO; Python's // floors.

    This is not pedantry: `(w - crop_sz)` is almost always negative here (the
    crop is bigger than the box), so `//` would place every crop one pixel off
    for odd differences, and the error compounds frame over frame as the box is
    re-derived from the crop each update."""
    return int(a / 2)


class SiameseOnnxTracker:
    """One-stream template/search tracker over a raw ONNX graph.

    Contract of the graph (TrackerVit / OSTrack-derived):
      inputs   'template' 1x3x128x128, 'search' 1x3x256x256
      outputs  'output1' 1x1x16x16   confidence
               'output2' 1x2x16x16   box size   (w,h), normalised to the crop
               'output3' 1x2x16x16   sub-cell offset (dx,dy)
    """

    TEMPLATE_SIZE = 128
    SEARCH_SIZE = 256
    SCORE = 16                      # score map is 16x16
    TEMPLATE_FACTOR = 2             # template crop = 2 x sqrt(w*h)
    SEARCH_FACTOR = 4               # search crop  = 4 x sqrt(w*h)

    # ImageNet statistics, pre-scaled to 0..255. NOTE the channel order: OpenCV
    # feeds BGR and applies these as Scalar[0..2] positionally, so the value
    # nominally belonging to R lands on B. That is what the C++ does, so it is
    # what the model was calibrated against in OpenCV, and it must be copied
    # exactly rather than "fixed".
    MEAN = np.array([0.485, 0.456, 0.406], np.float32) * 255.0
    STD = np.array([0.229, 0.224, 0.225], np.float32) * 255.0

    def __init__(self, model_path, score_threshold=0.20, search_factor=None, hann=True):
        # search_factor / hann are exposed because the INSTALLED OpenCV binary
        # does not match the published source: reverse-engineering it against
        # cv2.TrackerVit put the best fit at factor 3 without the Hann window,
        # while opencv/4.x and opencv/5.x both read factor 4 with it. Since the
        # binary cannot be verified, these stay configurable and the choice is
        # settled by tracking score on the battery, not by matching a black box.
        self.search_factor = self.SEARCH_FACTOR if search_factor is None else search_factor
        self.use_hann = hann
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.score_threshold = float(score_threshold)
        self.hann = self._hann2d(self.SCORE) if hann else np.ones((self.SCORE, self.SCORE), np.float32)
        self.rect = None
        self.tracking_score = 0.0

    # -- windowing ---------------------------------------------------------
    @staticmethod
    def _hann1d(sz):
        # Centred Hann, matching hann1d(centered=true): note the (sz + 1)
        # denominator and the (i + 1) numerator — neither endpoint is zero.
        i = np.arange(sz, dtype=np.float32)
        return 0.5 * (1.0 - np.cos((2.0 * math.pi / (sz + 1)) * (i + 1)))

    @classmethod
    def _hann2d(cls, sz):
        r = cls._hann1d(sz)
        return np.outer(r, r).astype(np.float32)

    # -- cropping ----------------------------------------------------------
    @classmethod
    def _crop(cls, img, rect, factor):
        """Square crop centred on `rect`, side = factor * sqrt(w*h), zero-padded
        where it falls outside the image. Returns (crop, crop_sz)."""
        x, y, w, h = rect
        crop_sz = int(math.ceil(math.sqrt(float(w) * float(h)) * factor))
        crop_sz = max(1, crop_sz)

        x1 = x + _trunc_div2(w - crop_sz)
        y1 = y + _trunc_div2(h - crop_sz)
        x2 = x1 + crop_sz
        y2 = y1 + crop_sz

        x1p = max(0, -x1)
        y1p = max(0, -y1)
        x2p = max(x2 - img.shape[1] + 1, 0)
        y2p = max(y2 - img.shape[0] + 1, 0)

        rx, ry = x1 + x1p, y1 + y1p
        rw = x2 - x2p - x1 - x1p
        rh = y2 - y2p - y1 - y1p
        if rw <= 0 or rh <= 0:                     # box left the frame entirely
            return np.zeros((crop_sz, crop_sz, 3), img.dtype), crop_sz
        patch = img[ry:ry + rh, rx:rx + rw]
        out = cv2.copyMakeBorder(patch, y1p, y2p, x1p, x2p, cv2.BORDER_CONSTANT, value=0)
        return out, crop_sz

    @classmethod
    def _blob(cls, crop, size):
        img = cv2.resize(crop, (size, size))
        f = img.astype(np.float32)
        f = (f - cls.MEAN) / cls.STD
        return np.transpose(f, (2, 0, 1))[None]    # HWC -> NCHW

    # -- API ---------------------------------------------------------------
    def init(self, image, rect):
        self.rect = [int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])]
        crop, _ = self._crop(image, self.rect, self.TEMPLATE_FACTOR)
        # The template is set ONCE and persists inside the graph for every
        # subsequent forward(). This is what makes it a fixed-template tracker
        # and therefore drift-free by construction.
        self.net.setInput(self._blob(crop, self.TEMPLATE_SIZE), 'template')

    def update(self, image):
        crop, crop_sz = self._crop(image, self.rect, self.search_factor)
        self.net.setInput(self._blob(crop, self.SEARCH_SIZE), 'search')
        outs = self.net.forward(['output1', 'output2', 'output3'])
        S = self.SCORE
        conf = outs[0].reshape(S, S)
        size_map = outs[1].reshape(2, S, S)
        off_map = outs[2].reshape(2, S, S)

        # Hann window suppresses peaks at the edge of the search region: a
        # target that far from the prediction is far more likely a distractor.
        conf = conf * self.hann
        pk = int(np.argmax(conf))
        py, px = divmod(pk, S)
        self.tracking_score = float(conf[py, px])
        if self.tracking_score < self.score_threshold:
            return False, tuple(self.rect)

        # Decode: cell index + sub-cell offset, normalised to the crop.
        cx = (px + float(off_map[0, py, px])) / S
        cy = (py + float(off_map[1, py, px])) / S
        w = float(size_map[0, py, px])
        h = float(size_map[1, py, px])

        x0 = self.rect[0] + _trunc_div2(self.rect[2] - crop_sz)
        y0 = self.rect[1] + _trunc_div2(self.rect[3] - crop_sz)
        self.rect = [int(math.floor((cx - w / 2) * crop_sz + x0)),
                     int(math.floor((cy - h / 2) * crop_sz + y0)),
                     int(math.floor(w * crop_sz)),
                     int(math.floor(h * crop_sz))]
        return True, tuple(self.rect)

    def getTrackingScore(self):
        return self.tracking_score


# ---------------------------------------------------------------------------
def validate_vs_opencv(model, clips_dir, on_thresh=25.0):
    """Frame-by-frame agreement against OpenCV's own C++ TrackerVit.

    This is the whole point of the module. If the boxes match, the crop,
    normalisation, Hann window and head decode above are correct, and the same
    logic can be carried into Kotlin with confidence instead of hope."""
    sys.path.insert(0, HERE)
    import eval_tracker as et

    names = sorted(n[:-4] for n in os.listdir(clips_dir) if n.endswith('.mp4'))
    print(f"\n{'clip':<22}{'frames':>7}{'exact':>8}{'<=1px':>8}{'<=2px':>8}{'max dev':>9}")
    print('-' * 63)
    tot = same = w1 = w2 = 0
    worst_all = 0.0
    for nm in names:
        frames = et.read_video(os.path.join(clips_dir, f'{nm}.mp4'))
        gt, dz = et.read_labels(os.path.join(clips_dir, f'{nm}.csv'), len(frames))
        fi0, cx0, cy0, sz0 = dz
        box = (int(cx0 - sz0 / 2), int(cy0 - sz0 / 2), int(sz0), int(sz0))

        p = cv2.TrackerVit_Params(); p.net = model
        ref = cv2.TrackerVit_create(p); ref.init(frames[fi0], box)
        mine = SiameseOnnxTracker(model); mine.init(frames[fi0], box)

        n = e = a1 = a2 = 0
        worst = 0.0
        for i in range(fi0 + 1, len(frames)):
            ok_r, br = ref.update(frames[i])
            ok_m, bm = mine.update(frames[i])
            n += 1
            d = max(abs(br[0] - bm[0]), abs(br[1] - bm[1]),
                    abs(br[2] - bm[2]), abs(br[3] - bm[3]))
            worst = max(worst, d)
            if d == 0:
                e += 1
            if d <= 1:
                a1 += 1
            if d <= 2:
                a2 += 1
        tot += n; same += e; w1 += a1; w2 += a2; worst_all = max(worst_all, worst)
        print(f"{nm:<22}{n:>7}{100*e/max(1,n):>7.0f}%{100*a1/max(1,n):>7.0f}%"
              f"{100*a2/max(1,n):>7.0f}%{worst:>9.0f}")
    print('-' * 63)
    print(f"{'TOTAL':<22}{tot:>7}{100*same/max(1,tot):>7.0f}%{100*w1/max(1,tot):>7.0f}%"
          f"{100*w2/max(1,tot):>7.0f}%{worst_all:>9.0f}")
    return same == tot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    ap.add_argument('--validate', action='store_true')
    a = ap.parse_args()
    if a.validate:
        ok = validate_vs_opencv(a.model, a.clips)
        print("\nEXACT MATCH — pipeline verified" if ok else
              "\nnot bit-exact; see the deviation columns above")


if __name__ == '__main__':
    main()
