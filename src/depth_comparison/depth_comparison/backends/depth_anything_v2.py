"""Depth Anything V2 metric depth backend.

One class covers every metric DA-V2 checkpoint. The checkpoint is what makes a
backend instance "vkitti" or "hypersim" — the architecture and inference path
are identical, only the training domain and ``max_depth`` differ:

======================  ==========  =========================================
checkpoint              max_depth   trained on
======================  ==========  =========================================
``vkitti``              80 m        Virtual KITTI — driving, camera at ~1.5 m
``hypersim``            20 m        Hypersim — indoor rooms
======================  ==========  =========================================

Note that DA-V2 metric models do **not** take intrinsics. Scale is baked into
the checkpoint from its training domain, which is exactly why a checkpoint
trained on car-mounted footage misjudges near-field geometry on ground-level
handheld video.
"""

import sys
from typing import Any

import numpy as np
import torch

from .base import DepthBackend, Intrinsics

#: Encoder variants and the feature widths the DA-V2 DPT head expects.
MODEL_CONFIGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}

#: ``max_depth`` each checkpoint was trained with, used when it is left unset.
#:
#: These are not free parameters. The DA-V2 metric head ends in
#: ``sigmoid(x) * max_depth``, so the value is a trained-in output scale —
#: overriding it rescales every prediction linearly rather than extending the
#: model's range. Change it only to match a checkpoint trained differently.
DATASET_MAX_DEPTH = {'vkitti': 80.0, 'hypersim': 20.0}

#: The metric variant of the repo — its DPT class accepts ``max_depth``, the
#: relative-depth one at the repo root does not.
_METRIC_REPO_PATH = '/opt/depth_anything_v2/metric_depth'


class DepthAnythingV2Backend(DepthBackend):
    """Depth Anything V2 metric depth, parameterised by checkpoint."""

    @staticmethod
    def param_defaults() -> dict[str, Any]:
        return {
            'dataset': 'vkitti',      # vkitti | hypersim — selects the checkpoint
            'encoder': 'vits',        # vits | vitb | vitl
            'max_depth': 0.0,         # 0 = take the default for `dataset`
            'model_path': '',         # empty = resolve from dataset + encoder
            'input_size': 518,        # DA-V2 internal inference resolution
        }

    def __init__(self, key, cfg, logger):
        super().__init__(key, cfg, logger)
        # Resolve configuration at construction, not in load(). describe() is
        # logged before the weights are loaded, and a bad encoder or dataset
        # should fail immediately rather than after a model download.
        self.encoder = cfg['encoder']
        if self.encoder not in MODEL_CONFIGS:
            raise ValueError(
                f'[{key}] unknown encoder "{self.encoder}", '
                f'expected one of {sorted(MODEL_CONFIGS)}'
            )

        self.dataset = cfg['dataset']
        max_depth = cfg['max_depth']
        if max_depth <= 0.0:
            if self.dataset not in DATASET_MAX_DEPTH:
                raise ValueError(
                    f'[{key}] no default max_depth for dataset "{self.dataset}"; '
                    f'set {key}.max_depth explicitly'
                )
            max_depth = DATASET_MAX_DEPTH[self.dataset]
        self.max_depth = max_depth

        self.input_size = int(cfg['input_size'])
        self.model_path = cfg['model_path'] or (
            f'/models/depth_anything_v2_metric_{self.dataset}_{self.encoder}.pth'
        )

    def load(self, device) -> None:
        if _METRIC_REPO_PATH not in sys.path:
            sys.path.insert(0, _METRIC_REPO_PATH)
        from depth_anything_v2.dpt import DepthAnythingV2

        self.device = device
        self.model = DepthAnythingV2(
            **MODEL_CONFIGS[self.encoder], max_depth=self.max_depth
        )

        self.logger.info(f'[{self.key}] loading weights from {self.model_path}')
        state_dict = torch.load(self.model_path, map_location=device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model = self.model.to(device).eval()

    def infer(self, bgr: np.ndarray, intrinsics: Intrinsics | None) -> np.ndarray:
        # infer_image() resizes internally and returns depth at the input
        # resolution, so the RGB CameraInfo stays valid for the result.
        with torch.no_grad():
            depth = self.model.infer_image(bgr, self.input_size)
        return depth.astype(np.float32)

    def describe(self) -> str:
        return (
            f'{self.key}: Depth Anything V2 metric  '
            f'dataset={self.dataset}  encoder={self.encoder}  '
            f'max_depth={self.max_depth}m  (intrinsics-agnostic)'
        )
