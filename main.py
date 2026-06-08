import argparse
import warnings
import time
import threading
import cv2
import torch

warnings.filterwarnings("ignore", message="xFormers is not available")

from tracker.embedding import DINOv2Embedder
from tracker.core import LockOnTracker, State
from tracker.ui import MouseHandler, draw_overlay
from tracker.settings import Settings, launch_settings
from tracker.pipeline import SharedState, capture_loop, inference_loop


def build_engine(engine: str, device: str, settings: Settings):
    """Pick the tracking philosophy. Returns (tracker, embedder_or_None)."""
    if engine == "sam2":
        from tracker.sam2_engine import SAM2Tracker
        print("Engine: SAM 2  (promptable mask propagation)")
        return SAM2Tracker(device=device), None
    embedder = DINOv2Embedder(device=device)
    print("Engine: DINOv2 hybrid  (box tracker + DINOv2 verification)")
    return LockOnTracker(embedder, settings=settings), embedder


def main():
    parser = argparse.ArgumentParser(description="Object lock-on tracker")
    parser.add_argument("--engine", choices=["hybrid", "sam2"], default="hybrid",
                        help="hybrid = DINOv2 + box tracker (default); "
                             "sam2 = SAM 2 mask propagation")
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA not available. "
            "Install the CUDA build of PyTorch:\n"
            "  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121"
        )
    device = "cuda"
    print(f"Device: cuda ({torch.cuda.get_device_name(0)})")

    settings = Settings()
    tracker, embedder = build_engine(args.engine, device, settings)
    mouse    = MouseHandler()

    launch_settings(settings)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open webcam at index 0")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    shared = SharedState()
    stop   = threading.Event()

    # Capture and inference run on their own threads; display stays on main.
    cap_thread = threading.Thread(
        target=capture_loop, args=(cap, shared, stop), daemon=True)
    inf_thread = threading.Thread(
        target=inference_loop,
        args=(tracker, embedder, settings, mouse, shared, stop), daemon=True)
    cap_thread.start()
    inf_thread.start()

    win = "Lock-On Tracker"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse.callback)

    disp_fps = 30.0
    t_prev   = time.perf_counter()

    try:
        while not stop.is_set():
            frame, _ = shared.get_frame()
            if frame is None:
                if cv2.waitKey(10) & 0xFF == 27:
                    break
                continue

            result = shared.get_result() or {}
            state        = result.get("state", State.IDLE)
            bbox         = result.get("bbox")
            sim          = result.get("sim", 0.0)
            attn_map     = result.get("attn_map")
            motion_trail = result.get("motion_trail")
            predicted    = result.get("predicted")

            t_now    = time.perf_counter()
            disp_fps = 0.9 * disp_fps + 0.1 / max(t_now - t_prev, 1e-9)
            t_prev   = t_now

            out = draw_overlay(
                frame, state, bbox, sim, mouse, disp_fps,
                settings=settings, attn_map=attn_map,
                motion_trail=motion_trail, predicted_center=predicted,
            )

            # Inference (tracking) rate, decoupled from display rate.
            inf_fps = result.get("inf_fps")
            if inf_fps and settings.show_fps:
                cv2.putText(out, f"{inf_fps:.0f} trk", (out.shape[1] - 76, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            cv2.imshow(win, out)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key in (ord("r"), ord("R")):
                shared.request_reset()
    finally:
        stop.set()
        cap_thread.join(timeout=1.0)
        inf_thread.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
