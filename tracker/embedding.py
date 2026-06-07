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
        DINOv2 block. Returns (soft_map, binary_mask) as float32 numpy arrays
        in [0, 1], resized to match crop_bgr.

        soft_map   — continuous heat map, good for blending overlays
        binary_mask — thresholded, good for contour extraction
        """
        h, w = crop_bgr.shape[:2]
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        tensor = self._transform(rgb).unsqueeze(0).to(self.device)

        # (1, num_heads, N+1, N+1)  where N = 256 patches for ViT-S/14 @ 224px
        attn = self.model.get_last_selfattention(tensor)

        nh = attn.shape[1]
        # CLS token attending to every patch: (num_heads, 256)
        cls_attn = attn[0, :, 0, 1:]

        # Reshape to spatial grid (16×16)
        side = int(cls_attn.shape[-1] ** 0.5)
        cls_attn = cls_attn.reshape(nh, side, side)

        # Average across heads, normalise to [0, 1]
        soft = cls_attn.mean(0)
        soft = (soft - soft.min()) / (soft.max() - soft.min() + 1e-8)

        soft_np = soft.cpu().float().numpy()
        soft_up = cv2.resize(soft_np, (w, h), interpolation=cv2.INTER_LINEAR)
        binary  = (soft_up >= threshold).astype(np.float32)

        return soft_up, binary
