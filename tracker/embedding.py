import cv2
import torch
import torch.nn.functional as F
from torchvision import transforms
import numpy as np


class DINOv2Embedder:
    """
    Wraps DINOv2-small (ViT-S/14) for instance-level embedding and
    attention-based foreground segmentation.
    """

    def __init__(self, device: str = "cuda"):
        self.device = device
        print("Loading DINOv2-small (ViT-S/14)…")
        self.model = torch.hub.load(
            "facebookresearch/dinov2", "dinov2_vits14", verbose=False
        )
        self.model.eval().to(device)
        self._transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ])
        print(f"DINOv2 ready on {device}")

    # ---------------------------------------------------------------- embedding

    @torch.inference_mode()
    def embed(self, crop_bgr: np.ndarray) -> torch.Tensor:
        """Normalised (384,) CLS feature vector from a BGR crop."""
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        tensor = self._transform(rgb).unsqueeze(0).to(self.device)
        feat = self.model(tensor)
        return F.normalize(feat, dim=-1).squeeze(0)

    @torch.inference_mode()
    def embed_batch(self, crops_bgr: list) -> torch.Tensor:
        """Embed many BGR crops in one GPU forward pass → (N, 384)."""
        if not crops_bgr:
            return torch.empty(0, 384, device=self.device)
        tensors = []
        for crop in crops_bgr:
            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            tensors.append(self._transform(rgb))
        batch = torch.stack(tensors).to(self.device)
        feats = self.model(batch)
        return F.normalize(feats, dim=-1)

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
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        tensor = self._transform(rgb).unsqueeze(0).to(self.device)

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
