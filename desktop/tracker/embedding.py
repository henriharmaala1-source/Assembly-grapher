import cv2
import torch
import torch.nn.functional as F
import numpy as np

_SIZE = 224
_MEAN = (0.485, 0.456, 0.406)
_STD  = (0.229, 0.224, 0.225)


class DINOv2Embedder:
    """
    Wraps DINOv2-small (ViT-S/14) for instance-level embedding and
    attention-based foreground segmentation.

    Performance:
      • FP16 autocast on CUDA (tensor cores ~2x throughput)
      • GPU preprocessing — cv2.resize per crop, one host->device transfer,
        normalisation done on the GPU (replaces slow per-crop PIL pipeline)
    """

    def __init__(self, device: str = "cuda"):
        self.device = device
        self._use_amp = (device == "cuda")
        print("Loading DINOv2-small (ViT-S/14)…")
        self.model = torch.hub.load(
            "facebookresearch/dinov2", "dinov2_vits14", verbose=False
        )
        self.model.eval().to(device)
        if device == "cuda":
            torch.backends.cudnn.benchmark = True          # fixed 224x224 input
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        # Normalisation constants as (1, 3, 1, 1) tensors on the device.
        self._mean = torch.tensor(_MEAN, device=device).view(1, 3, 1, 1)
        self._std  = torch.tensor(_STD,  device=device).view(1, 3, 1, 1)

        if device == "cuda":
            self._warmup()
        print(f"DINOv2 ready on {device}")

        self._frame_cache: "torch.Tensor | None" = None   # set by cache_frame()

    @torch.inference_mode()
    def _warmup(self):
        """Run a dummy forward so cudnn autotunes kernels before the live loop."""
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        self.embed_batch([dummy])
        torch.cuda.synchronize()

    # ------------------------------------------------- frame cache (GPU path)

    def cache_frame(self, frame_bgr: np.ndarray):
        """Upload the current frame to GPU once per inference loop iteration.
        Enables embed_boxes(), which slices and resizes crops on the GPU
        instead of running a Python loop of cv2 calls on the CPU.
        """
        t = torch.from_numpy(frame_bgr)
        if not t.is_contiguous():
            t = t.contiguous()
        self._frame_cache = t.to(self.device, non_blocking=True)

    @torch.inference_mode()
    def embed_boxes(self, boxes: list) -> torch.Tensor:
        """Embed a list of (x, y, w, h) boxes from the cached GPU frame.

        GPU-side crop + resize replaces the per-crop CPU cv2.cvtColor /
        cv2.resize loop in embed_batch(), giving ~3× throughput on large
        batches (e.g. SCAN_BATCH=64 during SEARCHING).
        Falls back to embed_batch() if no frame is cached.
        """
        if not boxes:
            return torch.empty(0, 384, device=self.device)
        if self._frame_cache is None:
            crops = []
            return torch.empty(0, 384, device=self.device)

        fh, fw = self._frame_cache.shape[:2]
        batch = torch.empty(len(boxes), 3, _SIZE, _SIZE,
                            dtype=torch.float32, device=self.device)

        for i, (x, y, w, h) in enumerate(boxes):
            x1, y1 = max(0, x),        max(0, y)
            x2, y2 = min(fw, x + w),   min(fh, y + h)
            if x2 <= x1 or y2 <= y1:
                batch[i].zero_()
                continue
            # Zero-copy GPU slice → float → (1,3,h,w) → bilinear to 224×224
            c = (self._frame_cache[y1:y2, x1:x2]
                 .float().permute(2, 0, 1).unsqueeze(0))
            batch[i] = F.interpolate(
                c, (_SIZE, _SIZE), mode="bilinear", align_corners=False
            ).squeeze(0)

        # BGR → RGB channel swap, normalise to [0,1], then ImageNet z-score
        batch = batch[:, [2, 1, 0]] / 255.0
        batch = (batch - self._mean) / self._std

        with torch.autocast("cuda", dtype=torch.float16, enabled=self._use_amp):
            feats = self.model(batch)
        return F.normalize(feats.float(), dim=-1)

    # ------------------------------------------------------------ preprocessing

    def _preprocess(self, crops_bgr: list) -> torch.Tensor:
        """
        BGR numpy crops → normalised (N, 3, 224, 224) tensor on device.
        cv2 handles colour + resize on CPU (SIMD), then a single transfer and
        GPU-side normalisation.
        """
        arr = np.empty((len(crops_bgr), _SIZE, _SIZE, 3), dtype=np.uint8)
        for i, crop in enumerate(crops_bgr):
            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            arr[i] = cv2.resize(rgb, (_SIZE, _SIZE), interpolation=cv2.INTER_LINEAR)
        t = torch.from_numpy(arr).to(self.device, non_blocking=True)
        t = t.permute(0, 3, 1, 2).float().div_(255.0)      # (N,3,224,224)
        return (t - self._mean) / self._std

    # ---------------------------------------------------------------- embedding

    @torch.inference_mode()
    def embed(self, crop_bgr: np.ndarray) -> torch.Tensor:
        """Normalised (384,) CLS feature vector from a BGR crop."""
        return self.embed_batch([crop_bgr]).squeeze(0)

    @torch.inference_mode()
    def embed_batch(self, crops_bgr: list) -> torch.Tensor:
        """Embed many BGR crops in one GPU forward pass → (N, 384)."""
        if not crops_bgr:
            return torch.empty(0, 384, device=self.device)
        batch = self._preprocess(crops_bgr)
        with torch.autocast("cuda", dtype=torch.float16, enabled=self._use_amp):
            feats = self.model(batch)
        return F.normalize(feats.float(), dim=-1)

    def similarity(self, a: torch.Tensor, b: torch.Tensor) -> float:
        return F.cosine_similarity(a.unsqueeze(0), b.unsqueeze(0)).item()

    # --------------------------------------------------------- attention mask

    @torch.inference_mode()
    def get_attention_map(
        self,
        crop_bgr: np.ndarray,
        threshold: float = 0.5,
    ):
        """
        Extract a foreground mask from the CLS-to-patch attention of the last
        DINOv2 block. Recent DINOv2 builds use fused scaled-dot-product
        attention, so there is no attention tensor to hook directly. Instead
        we hook the qkv linear (always a real module), capture its output, and
        recompute the CLS->patch attention ourselves.
        Returns (soft_map, binary_mask) as float32 numpy in [0,1], sized to crop_bgr.
        """
        h, w = crop_bgr.shape[:2]
        # Single-image preprocess; keep FP32 here for clean softmax numerics.
        tensor = self._preprocess([crop_bgr])

        attn_mod = self.model.blocks[-1].attn
        captured = {}

        def _hook(module, inp, out):
            captured["qkv"] = out.detach()      # (B, N, 3*C)

        handle = attn_mod.qkv.register_forward_hook(_hook)
        try:
            self.model(tensor)
        finally:
            handle.remove()

        if "qkv" not in captured:
            blank = np.full((h, w), 0.5, dtype=np.float32)
            return blank, (blank >= threshold).astype(np.float32)

        qkv = captured["qkv"]
        B, N, C3 = qkv.shape
        C = C3 // 3
        nh = attn_mod.num_heads
        head_dim = C // nh
        scale = head_dim ** -0.5

        # (3, B, nh, N, head_dim)
        qkv = qkv.reshape(B, N, 3, nh, head_dim).permute(2, 0, 3, 1, 4)
        q, k = qkv[0], qkv[1]                    # (B, nh, N, head_dim)

        attn = (q * scale) @ k.transpose(-2, -1)   # (B, nh, N, N)
        attn = attn.softmax(dim=-1)

        # CLS token (row 0) attending to every other token: (nh, N-1)
        cls_attn = attn[0, :, 0, 1:]

        # Trim to trailing perfect-square patch tokens (tolerates *_reg models)
        patches = cls_attn.shape[-1]
        side = int(patches ** 0.5)
        cls_attn = cls_attn[:, -side * side:].reshape(nh, side, side)  # (nh,s,s)

        # Weight each head by inverse entropy — focused heads (low entropy)
        # carry more spatial information and should dominate the mask.
        flat    = cls_attn.reshape(nh, -1).clamp(min=1e-8)
        entropy = -(flat * flat.log()).sum(-1)          # (nh,) lower = sharper
        weights = 1.0 / (entropy + 1e-6)
        weights = weights / weights.sum()
        soft    = (cls_attn * weights[:, None, None]).sum(0)   # (s, s)
        soft    = (soft - soft.min()) / (soft.max() - soft.min() + 1e-8)

        soft_np = soft.cpu().float().numpy()
        soft_up = cv2.resize(soft_np, (w, h), interpolation=cv2.INTER_LINEAR)

        # Otsu automatic threshold (ignores the manual slider value) unless
        # the image is near-uniform, in which case fall back to the slider.
        soft_u8 = (soft_up * 255).astype(np.uint8)
        otsu_thresh, binary_otsu = cv2.threshold(
            soft_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        if otsu_thresh > 0:
            binary = (binary_otsu / 255).astype(np.float32)
        else:
            binary = (soft_up >= threshold).astype(np.float32)

        return soft_up, binary
