"""Metric3D V2 metric depth backend.

Unlike Depth Anything V2, Metric3D V2 is **intrinsics-conditioned**: it predicts
depth in a canonical camera space with a fixed 1000 px focal length, and metric
scale is recovered afterwards from the real camera's focal length. That makes it
the interesting contrast in this comparison — the DA-V2 checkpoints inherit
their scale from a training domain, Metric3D takes it from the camera.

The pre/post-processing below (canonical resize, mean-value padding, ImageNet
normalisation, un-pad, de-canonical rescale) follows the reference inference
procedure from the upstream repo. Deviating from it silently breaks metric
scale, so keep the steps in this order.

``monocular_depth``'s depth node — where Metric3D now runs in production —
carries the same procedure. The duplication keeps the production node free of a
dependency on this comparison harness, but a fix here belongs there too.
"""

from typing import Any

import cv2
import numpy as np
import torch

from .base import DepthBackend, Intrinsics

#: Input resolution the ViT variants are trained at, as (height, width).
_CANONICAL_INPUT_SIZE = (616, 1064)

#: Focal length of the canonical camera Metric3D predicts in, in pixels.
_CANONICAL_FOCAL = 1000.0

#: ImageNet statistics in RGB order, as used upstream. The padding value is the
#: channel mean, so padded borders normalise to zero.
_MEAN = np.array([123.675, 116.28, 103.53], dtype=np.float32)
_STD = np.array([58.395, 57.12, 57.375], dtype=np.float32)


class Metric3DV2Backend(DepthBackend):
    """Metric3D V2 depth, scaled to metres using the live camera intrinsics."""

    @staticmethod
    def param_defaults() -> dict[str, Any]:
        return {
            'model_name': 'metric3d_vit_small',   # _vit_small | _vit_large | _vit_giant2
            'repo': 'yvanyin/metric3d',
            'max_depth': 300.0,                   # clamp on the de-canonicalised output
        }

    def load(self, device) -> None:
        self.device = device
        self.max_depth = float(self.cfg['max_depth'])
        model_name = self.cfg['model_name']

        self.logger.info(f'[{self.key}] loading {model_name} via torch.hub')
        # Weights are pre-fetched into the torch hub cache at image build time,
        # so this resolves offline in the container.
        self.model = torch.hub.load(
            self.cfg['repo'], model_name, pretrain=True, trust_repo=True
        )
        self.model = self.model.to(device).eval()

    def infer(self, bgr: np.ndarray, intrinsics: Intrinsics | None) -> np.ndarray:
        if intrinsics is None:
            raise RuntimeError(
                f'[{self.key}] Metric3D recovers metric scale from the camera '
                'focal length; no CameraInfo has arrived yet.'
            )

        h, w = bgr.shape[:2]
        rgb = bgr[:, :, ::-1]  # upstream normalisation statistics are RGB-ordered

        # ── Resize into canonical space, preserving aspect ratio ─────────────
        scale = min(_CANONICAL_INPUT_SIZE[0] / h, _CANONICAL_INPUT_SIZE[1] / w)
        resized = cv2.resize(
            rgb, (int(round(w * scale)), int(round(h * scale))),
            interpolation=cv2.INTER_LINEAR,
        )

        # ── Pad to the exact canonical size with the channel means ──────────
        rh, rw = resized.shape[:2]
        pad_h, pad_w = _CANONICAL_INPUT_SIZE[0] - rh, _CANONICAL_INPUT_SIZE[1] - rw
        pad_top, pad_left = pad_h // 2, pad_w // 2
        pad_bottom, pad_right = pad_h - pad_top, pad_w - pad_left
        padded = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=_MEAN.tolist(),
        )

        # ── Normalise and run ───────────────────────────────────────────────
        tensor = torch.from_numpy(
            ((padded.astype(np.float32) - _MEAN) / _STD).transpose(2, 0, 1)
        )[None].to(self.device)

        with torch.no_grad():
            pred_depth, _confidence, _outputs = self.model.inference({'input': tensor})

        # ── Undo the padding, then return to the source resolution ──────────
        pred_depth = pred_depth.squeeze()
        pred_depth = pred_depth[
            pad_top:pred_depth.shape[0] - pad_bottom,
            pad_left:pred_depth.shape[1] - pad_right,
        ]
        pred_depth = torch.nn.functional.interpolate(
            pred_depth[None, None], (h, w), mode='bilinear', align_corners=False,
        ).squeeze()

        # ── De-canonicalise: canonical focal 1000 px → this camera's focal ──
        # `intrinsics.fx` is for the source image; the same `scale` that took
        # the image into canonical space applies to the focal length.
        canonical_to_real = (intrinsics.fx * scale) / _CANONICAL_FOCAL
        pred_depth = torch.clamp(pred_depth * canonical_to_real, 0.0, self.max_depth)

        return pred_depth.cpu().numpy().astype(np.float32)

    def describe(self) -> str:
        return (
            f'{self.key}: Metric3D V2  model={self.cfg["model_name"]}  '
            f'max_depth={self.cfg["max_depth"]}m  (intrinsics-conditioned)'
        )
