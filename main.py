import time
import cv2
import torch

from tracker.embedding import DINOv2Embedder
from tracker.core import LockOnTracker, State
from tracker.ui import MouseHandler, draw_overlay


def main():
    if torch.cuda.is_available():
        device = "cuda"
        print(f"Device: cuda ({torch.cuda.get_device_name(0)})")
    else:
        device = "cpu"
        print("Device: cpu  *** CUDA not available — performance will be low ***")
        print("  Fix: pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121")
        print("  Then verify: python -c \"import torch; print(torch.cuda.is_available())\"")
        print()

    embedder = DINOv2Embedder(device=device)
    tracker  = LockOnTracker(embedder)
    mouse    = MouseHandler()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open webcam at index 0")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    win = "Lock-On Tracker"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse.callback)

    fps    = 30.0
    t_prev = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Consume a freshly drawn bounding box
        if mouse.pending_bbox is not None and tracker.state == State.IDLE:
            tracker.init(frame, mouse.pending_bbox)
            mouse.pending_bbox = None

        state, bbox, sim = tracker.update(frame)

        t_now = time.perf_counter()
        fps   = 0.9 * fps + 0.1 / max(t_now - t_prev, 1e-9)
        t_prev = t_now

        out = draw_overlay(frame, state, bbox, sim or 0.0, mouse, fps)
        cv2.imshow(win, out)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:                          # ESC → quit
            break
        elif key in (ord("r"), ord("R")):      # R → reset lock
            tracker.reset()
            mouse.pending_bbox = None

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
