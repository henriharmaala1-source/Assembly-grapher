import argparse
import warnings
import time
import threading
import cv2
import torch

warnings.filterwarnings("ignore", message="xFormers is not available")

from tracker.core import State
from tracker.ui import MouseHandler, draw_overlay
from tracker.settings import Settings, launch_settings
from tracker.pipeline import SharedState, capture_loop, inference_loop
from tracker.engine import EngineManager
from tracker.click_segment import ClickSegmenter


def _banner(out, text, color):
    """Centered status banner (engine loading / errors)."""
    fh, fw = out.shape[:2]
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    x, y = (fw - tw) // 2, fh // 2
    cv2.rectangle(out, (x - 12, y - th - 10), (x + tw + 12, y + 12), (0, 0, 0), -1)
    cv2.putText(out, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    parser = argparse.ArgumentParser(description="Object lock-on tracker")
    parser.add_argument("--engine", choices=["hybrid", "sam2"], default="hybrid",
                        help="initial engine; switchable live in the settings "
                             "window. hybrid = DINOv2 + box tracker (default); "
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
    settings.tracking_engine = args.engine
    manager = EngineManager(device, settings)
    try:
        manager.get(args.engine)          # build the initial engine up front
    except Exception as e:
        print(f"[warning] could not load '{args.engine}' engine: {e}")
        print("[warning] falling back to hybrid engine.")
        settings.tracking_engine = "hybrid"
        manager.get("hybrid")
    segmenter = ClickSegmenter(device, settings)
    mouse     = MouseHandler()

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
        args=(manager, segmenter, settings, mouse, shared, stop), daemon=True)
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
            seg_result   = result.get("seg_result")
            motion_blobs = result.get("motion_blobs")
            motion_mask  = result.get("motion_mask")

            t_now    = time.perf_counter()
            disp_fps = 0.9 * disp_fps + 0.1 / max(t_now - t_prev, 1e-9)
            t_prev   = t_now

            out = draw_overlay(
                frame, state, bbox, sim, mouse, disp_fps,
                settings=settings, attn_map=attn_map,
                motion_trail=motion_trail, predicted_center=predicted,
                seg_result=seg_result,
                motion_blobs=motion_blobs, motion_mask=motion_mask,
            )

            # Inference (tracking) rate, decoupled from display rate.
            inf_fps = result.get("inf_fps")
            if inf_fps and settings.show_fps:
                cv2.putText(out, f"{inf_fps:.0f} trk", (out.shape[1] - 76, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # Engine-switch feedback.
            loading = result.get("engine_loading")
            err     = result.get("engine_error")
            if loading:
                _banner(out, f"Loading {loading} engine…", (0, 200, 255))
            elif err:
                _banner(out, f"Engine load failed: {err[:48]}", (40, 40, 240))

            cv2.imshow(win, out)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key in (ord("r"), ord("R")):
                shared.request_reset()
            elif key in (ord("c"), ord("C")):
                # clear the segment overlay without resetting tracking
                mouse.pending_point = None
                result = shared.get_result() or {}
                result.pop("seg_result", None)
                shared.set_result(result)
    finally:
        stop.set()
        cap_thread.join(timeout=1.0)
        inf_thread.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
