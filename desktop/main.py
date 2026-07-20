import argparse
import os
import warnings
import time
import threading
import cv2
import torch

warnings.filterwarnings("ignore", message="xFormers is not available")

from tracker.core import State
from tracker.ui import MouseHandler, draw_overlay
from tracker.settings import Settings, launch_settings, FILTER_MODES, PIP_FILTERS
from tracker.pipeline import SharedState, capture_loop, inference_loop, cpu_inference_loop
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
    parser.add_argument("--depth-model", default="",
                        help="path to ONNX depth model (MiDaS Small or DAv2 Small)")
    parser.add_argument("--depth-backend", choices=["midas", "dav2"], default="midas",
                        help="depth model type: midas (default) or dav2")
    parser.add_argument("--depth-interval", type=int, default=6,
                        help="run depth every N tracking frames (default 6)")
    parser.add_argument("--depth-on", action="store_true",
                        help="start with depth overlay visible")
    parser.add_argument("--cpu", action="store_true",
                        help="force CPU-only mode (depth nav + drone CV tracking; "
                             "no DINOv2/SAM2). Auto-enabled when CUDA is missing.")
    args = parser.parse_args()

    # Resolve depth model path — search order:
    #   1. exactly as given (absolute path or ./models/... that already exists)
    #   2. DEPTH_MODELS env var folder   e.g. set DEPTH_MODELS=C:\Users\you\depth_models
    #   3. ~/depth_models/               persistent across project re-downloads
    #   4. ./models/                     project-local fallback
    depth_model = args.depth_model
    if depth_model and not os.path.isfile(depth_model):
        basename = os.path.basename(depth_model)
        search = [
            os.path.join(os.environ.get("DEPTH_MODELS", ""), basename),
            os.path.join(os.path.expanduser("~"), "depth_models", basename),
            os.path.join("models", basename),
        ]
        for candidate in search:
            if candidate and os.path.isfile(candidate):
                print(f"[depth] resolved model: {candidate}")
                depth_model = candidate
                break
        else:
            print(f"[depth] model not found: {args.depth_model}")
            print(f"[depth] searched: {[s for s in search if s]}")
            print("[depth] download it once to ~/depth_models/ to keep it across updates:")
            print("  curl.exe -L -o %USERPROFILE%\\depth_models\\midas_small.onnx "
                  "https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx")
            depth_model = ""

    cpu_mode = args.cpu or not torch.cuda.is_available()
    if cpu_mode and not args.cpu:
        print("[warning] CUDA not available in this Python — running CPU-only mode.")
        print("[warning] Depth nav + drone CV tracking work; DINOv2 / SAM 2 are disabled.")
        print("[warning] For GPU lock-on, install the CUDA build of PyTorch:")
        print("[warning]   pip install torch torchvision "
              "--index-url https://download.pytorch.org/whl/cu121")

    settings = Settings()
    settings.tracking_engine = args.engine
    settings.depth_model    = depth_model
    settings.depth_backend  = args.depth_backend
    settings.depth_interval = args.depth_interval
    settings.depth_on       = args.depth_on

    manager = segmenter = None
    if not cpu_mode:
        device = "cuda"
        print(f"Device: cuda ({torch.cuda.get_device_name(0)})")
        manager = EngineManager(device, settings)
        try:
            manager.get(args.engine)          # build the initial engine up front
        except Exception as e:
            print(f"[warning] could not load '{args.engine}' engine: {e}")
            print("[warning] falling back to hybrid engine.")
            settings.tracking_engine = "hybrid"
            manager.get("hybrid")
        segmenter = ClickSegmenter(device, settings)

    mouse = MouseHandler()

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
        target=capture_loop, args=(cap, shared, stop, settings), daemon=True)
    if cpu_mode:
        inf_thread = threading.Thread(
            target=cpu_inference_loop,
            args=(settings, mouse, shared, stop), daemon=True)
    else:
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
            drone_result        = result.get("drone_result")
            drone_detections    = result.get("drone_detections") or []
            drone_detect_preset = result.get("drone_detect_preset", "Drone-vs-Bird")
            depth_snap          = result.get("depth_snap")

            t_now    = time.perf_counter()
            disp_fps = 0.9 * disp_fps + 0.1 / max(t_now - t_prev, 1e-9)
            t_prev   = t_now

            out = draw_overlay(
                frame, state, bbox, sim, mouse, disp_fps,
                settings=settings, attn_map=attn_map,
                motion_trail=motion_trail, predicted_center=predicted,
                seg_result=seg_result,
                motion_blobs=motion_blobs, motion_mask=motion_mask,
                drone_result=drone_result,
                drone_detections=drone_detections,
                drone_detect_preset=drone_detect_preset,
                depth_snap=depth_snap,
            )

            # Inference (tracking) rate, decoupled from display rate.
            inf_fps = result.get("inf_fps")
            if inf_fps and settings.show_fps:
                cv2.putText(out, f"{inf_fps:.0f} trk", (out.shape[1] - 76, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # Zoom / filter status tag (only when either is active).
            if settings.zoom > 1.001 or settings.filter_mode != "none":
                tag = f"zoom {settings.zoom:.2f}x   filter {settings.filter_mode}"
                cv2.putText(out, tag, (12, out.shape[0] - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

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
            elif key in (ord("d"), ord("D")) and not cpu_mode:
                settings.drone_mode = not settings.drone_mode
            elif key in (ord("n"), ord("N")):
                settings.depth_on = not settings.depth_on
            elif key in (ord("+"), ord("=")):
                settings.zoom = min(4.0, round(settings.zoom + 0.25, 2))
            elif key in (ord("-"), ord("_")):
                settings.zoom = max(1.0, round(settings.zoom - 0.25, 2))
            elif key in (ord("f"), ord("F")):
                order = FILTER_MODES
                settings.filter_mode = order[
                    (order.index(settings.filter_mode) + 1) % len(order)]
            elif key in (ord("g"), ord("G")):
                order = PIP_FILTERS
                settings.zoom_pip_filter = order[
                    (order.index(settings.zoom_pip_filter) + 1) % len(order)]
            elif key in (ord("p"), ord("P")):
                settings.zoom_pip = not settings.zoom_pip
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
