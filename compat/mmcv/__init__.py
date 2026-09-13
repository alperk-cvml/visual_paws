"""Minimal ``mmcv`` 1.x compatibility shim, backed by mmengine.

Metric3D V2 imports mmcv 1.x on its inference path — ``mono/utils/comm.py``
does ``from mmcv.utils import collect_env`` unconditionally, and mmcv 1.x is
not installable here (it predates Python 3.12 and fails at metadata
generation). Real mmcv 2.x is no help either: the ``mmcv.utils`` namespace was
removed and its contents moved into mmengine.

The surface Metric3D actually needs is four symbols, all of which mmengine
provides. This package supplies exactly those and nothing else, so an upstream
change that reaches for more fails loudly rather than silently misbehaving.

This is deliberately a compatibility shim and not a patch to Metric3D — the
upstream checkout in the torch hub cache stays untouched, matching the repo
rule against editing vendored third-party code.

Note that this package sits on ``PYTHONPATH`` and will therefore shadow a real
``mmcv`` if one is ever installed. If upstream ships a Python 3.12-compatible
mmcv, delete this directory and the ``COPY``/``ENV`` lines in the Dockerfile
rather than leaving both in place.
"""

from . import utils  # noqa: F401  (importing binds the mmcv.utils submodule)

#: Marks the shim clearly in any environment dump or bug report.
__version__ = '1.7.2+mmengine-shim'

__all__ = ['utils']
