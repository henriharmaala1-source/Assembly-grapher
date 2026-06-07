import time
import cv2
import torch

from tracker.embedding import DINOv2Embedder
from tracker.core import LockOnTracker, State
from tracker.ui import MouseHandler, draw_overlay
from tracker.settings import Settings, launch_settings


def main():
    if torch.cuda.is_available():
        device = "cuda"
        print(f"Device: cuda ({torch.cuda.get_device_name(0)})")
    else:
        device = "cpu"
        print("Device: cpu  *** CUDA not available — performance will be low ***")
        print("  Fix: pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121")
        print()

    embedder = DINOv2Embedder(device=device)
    settings = Settings()
    tracker  = LockOnTracker(embedder, settings=settings)
    mouse    = MouseHandler()

    launch_settings(settings)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open webcam at index 0")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    win = "Lock-On Tracker"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse.callback)

    fps      = 30.0
    t_prev   = time.perf_counter()
    attn_map = None           # cached attention map (reused between frames)
    attn_age = 0              # frames since last attention computation

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if mouse.pending_bbox is not None and tracker.state == State.IDLE:
            tracker.init(frame, mouse.pending_bbox)
            mouse.pending_bbox = None
            attn_map = None
            attn_age = 0

        state, bbox, sim = tracker.update(frame)

        # Recompute attention map every attn_interval frames when locked and mask is on
        if settings.show_mask and state == State.LOCKED and bbox is not None:
            attn_age += 1
            if attn_map is None or attn_age >= settings.attn_interval:
                x, y, w, h = bbox
                fh, fw = frame.shape[:2]
                crop = frame[max(0,y):min(fh,y+h), max(0,x):min(fw,x+w)]
                if crop.size > 0:
                    attn_map, _ = embedder.get_attention_map(
                        crop, threshold=settings.attn_threshold
                    )
                    attn_age = 0
        else:
            attn_map = None
            attn_age = 0

        t_now  = time.perf_counter()
        fps    = 0.9 * fps + 0.1 / max(t_now - t_prev, 1e-9)
        t_prev = t_now

        out = draw_overlay(
            frame, state, bbox, sim or 0.0, mouse, fps,
            settings=settings, attn_map=attn_map,
        )
        cv2.imshow(win, out)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key in (ord("r"), ord("R")):
            tracker.reset()
            mouse.pending_bbox = None
            attn_map = None

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
