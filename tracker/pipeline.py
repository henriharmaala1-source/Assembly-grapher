import time
import threading
from .core import State


class SharedState:
    """
    Thread-safe hand-off between the capture, inference and display threads.

    - Capture thread writes the latest frame (old frames are dropped).
    - Inference thread reads the latest frame and writes the latest result.
    - Display thread (main) reads both for rendering.
    Control signals (reset) flow from the display thread to the inference thread.
    """

    def __init__(self):
        self._flock  = threading.Lock()
        self._frame  = None
        self._fseq   = 0

        self._rlock  = threading.Lock()
        self._result = None

        self._reset  = threading.Event()

    # --- frames -------------------------------------------------------------

    def set_frame(self, frame):
        with self._flock:
            self._frame = frame
            self._fseq += 1

    def get_frame(self):
        """Returns (frame_or_None, seq)."""
        with self._flock:
            return self._frame, self._fseq

    # --- results ------------------------------------------------------------

    def set_result(self, result: dict):
        with self._rlock:
            self._result = result

    def get_result(self):
        with self._rlock:
            return self._result

    # --- control ------------------------------------------------------------

    def request_reset(self):
        self._reset.set()

    def take_reset(self) -> bool:
        if self._reset.is_set():
            self._reset.clear()
            return True
        return False


def capture_loop(cap, shared: SharedState, stop: threading.Event):
    """Continuously grab frames; only the newest is kept."""
    while not stop.is_set():
        ok, frame = cap.read()
        if not ok:
            print("Camera read failed — stopping.")
            stop.set()
            break
        shared.set_frame(frame)


def _compute_attention(embedder, frame, bbox, settings):
    x, y, w, h = bbox
    fh, fw = frame.shape[:2]
    crop = frame[max(0, y):min(fh, y + h), max(0, x):min(fw, x + w)]
    if crop.size == 0:
        return None
    attn, _ = embedder.get_attention_map(crop, threshold=settings.attn_threshold)
    return attn


def _switch_engine(manager, want, active_name, tracker, embedder, frame, shared):
    """Swap to engine `want`. On failure keep the current engine and revert the
    setting so we don't retry every frame. Returns (tracker, embedder, name)."""
    # Tell the display a (possibly slow) load is happening.
    shared.set_result(dict(state=tracker.state, bbox=getattr(tracker, "bbox", None),
                           sim=0.0, engine_loading=want))
    try:
        new_tracker, new_embedder = manager.get(want)
    except Exception as e:
        print(f"[engine] could not load '{want}': {e}")
        manager.settings.tracking_engine = active_name      # revert dropdown intent
        shared.set_result(dict(state=tracker.state, engine_error=str(e)))
        return tracker, embedder, active_name

    carry = tracker.bbox if tracker.state == State.LOCKED else None
    new_tracker.reset()
    if carry is not None:
        try:
            new_tracker.init(frame, carry)                  # re-prompt, no re-draw
        except Exception:
            pass
    print(f"[engine] switched to '{want}'")
    return new_tracker, new_embedder, want


def inference_loop(manager, settings, mouse, shared: SharedState,
                   stop: threading.Event):
    """Run the active engine on the newest frame; publish results for display.

    The engine (DINOv2 hybrid / SAM 2) can be switched live from the settings
    window; the swap happens here on the inference thread, carrying over the
    current lock box so tracking continues without re-drawing.
    """
    active_name = settings.tracking_engine
    tracker, embedder = manager.get(active_name)
    last_seq  = -1
    attn_map  = None
    attn_age  = 0
    inf_fps   = 30.0
    t_prev    = time.perf_counter()

    while not stop.is_set():
        frame, seq = shared.get_frame()
        if frame is None or seq == last_seq:
            time.sleep(0.001)        # nothing new yet
            continue
        last_seq = seq

        # --- live engine switch -------------------------------------------
        want = settings.tracking_engine
        if want != active_name:
            tracker, embedder, active_name = _switch_engine(
                manager, want, active_name, tracker, embedder, frame, shared)
            attn_map = None
            attn_age = 0

        if shared.take_reset():
            tracker.reset()
            mouse.pending_bbox = None
            attn_map = None
            attn_age = 0

        if mouse.pending_bbox is not None and tracker.state == State.IDLE:
            tracker.init(frame, mouse.pending_bbox)
            mouse.pending_bbox = None
            attn_map = None
            attn_age = 0

        state, bbox, sim = tracker.update(frame)

        if settings.show_mask and state == State.LOCKED and bbox is not None:
            if getattr(tracker, "provides_mask", False):
                # SAM 2 already produced a pixel-accurate mask this frame.
                attn_map = tracker.mask_crop()
            else:
                attn_age += 1
                if attn_map is None or attn_age >= settings.attn_interval:
                    attn_map = _compute_attention(embedder, frame, bbox, settings)
                    attn_age = 0
        else:
            attn_map = None
            attn_age = 0

        motion_trail = tracker.center_trail() if settings.show_motion_vector else None
        predicted    = tracker.predicted_center if settings.show_motion_vector else None

        t_now   = time.perf_counter()
        inf_fps = 0.9 * inf_fps + 0.1 / max(t_now - t_prev, 1e-9)
        t_prev  = t_now

        shared.set_result(dict(
            state=state, bbox=bbox, sim=sim or 0.0,
            attn_map=attn_map, motion_trail=motion_trail,
            predicted=predicted, inf_fps=inf_fps,
        ))
