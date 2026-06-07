import cv2
import torch
import torch.nn.functional as F
from torchvision import transforms
import numpy as np


class DINOv2Embedder:
    """
    Wraps DINOv2-small (ViT-S/14) for instance-level embedding.
    Outputs a 384-dim L2-normalised CLS token per crop.
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

    @torch.inference_mode()
    def embed(self, crop_bgr: np.ndarray) -> torch.Tensor:
        """Return a normalised (384,) feature vector from a BGR numpy crop."""
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        tensor = self._transform(rgb).unsqueeze(0).to(self.device)
        feat = self.model(tensor)                        # (1, 384)
        return F.normalize(feat, dim=-1).squeeze(0)     # (384,)

    @torch.inference_mode()
    def embed_batch(self, crops_bgr: list) -> torch.Tensor:
        """
        Embed many BGR crops in a single forward pass.
        Returns a normalised (N, 384) tensor on self.device.
        """
        if not crops_bgr:
            return torch.empty(0, 384, device=self.device)
        tensors = []
        for crop in crops_bgr:
            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            tensors.append(self._transform(rgb))
        batch = torch.stack(tensors).to(self.device)     # (N, 3, 224, 224)
        feats = self.model(batch)                        # (N, 384)
        return F.normalize(feats, dim=-1)

    def similarity(self, a: torch.Tensor, b: torch.Tensor) -> float:
        return F.cosine_similarity(a.unsqueeze(0), b.unsqueeze(0)).item()
