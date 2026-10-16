"""
Engine manager — lets the app switch tracking *philosophies* at runtime.

Each engine (DINOv2 hybrid, SAM 2) is built lazily on first use and cached, so
toggling between them in the settings window is instant after the initial load.
Both are small enough to sit in VRAM together on a modern GPU.
"""


class EngineManager:
    def __init__(self, device: str, settings):
        self.device   = device
        self.settings = settings
        self._cache   = {}     # name -> (tracker, embedder_or_None)

    def get(self, name: str):
        """Return (tracker, embedder); build + cache on first request.

        May raise if the engine's dependencies/checkpoints are missing — the
        caller is expected to handle that and keep the previous engine.
        """
        if name not in self._cache:
            self._cache[name] = self._build(name)
        return self._cache[name]

    def is_built(self, name: str) -> bool:
        return name in self._cache

    def _build(self, name: str):
        if name == "sam2":
            from .sam2_engine import SAM2Tracker
            print("Loading engine: SAM 2  (promptable mask propagation)…")
            return SAM2Tracker(device=self.device), None

        from .embedding import DINOv2Embedder
        from .core import LockOnTracker
        print("Loading engine: DINOv2 hybrid  (box tracker + DINOv2)…")
        embedder = DINOv2Embedder(device=self.device)
        return LockOnTracker(embedder, settings=self.settings), embedder
