"""The four ``mmcv.utils`` symbols Metric3D V2 imports, re-exported from mmengine.

Determined by grepping the upstream checkout rather than guessed:

    collect_env    mono/utils/comm.py  (unconditional, on the inference path)
    get_git_hash   mono/utils/comm.py
    Config         hubconf.py, mono/model/backbones/ViT_DINO*.py, decoders
    DictAction     mono/tools/*.py

mmengine is the successor project and exposes all four, so these are genuine
re-exports rather than reimplementations — behaviour matches whatever mmengine
version is installed.
"""

from mmengine import Config, DictAction
from mmengine.utils import get_git_hash
from mmengine.utils.dl_utils import collect_env

__all__ = ['Config', 'DictAction', 'collect_env', 'get_git_hash']


def __getattr__(name):
    """Fail loudly on anything this shim does not actually cover.

    Dunder lookups must raise AttributeError, not ImportError: the import
    machinery probes ``__path__`` and friends while resolving
    ``from mmcv.utils import ...``, and an ImportError there aborts the import
    with a misleading message.
    """
    if name.startswith('__'):
        raise AttributeError(name)
    raise ImportError(
        f'mmcv.utils.{name} is not provided by the mmengine compatibility '
        f'shim (see {__package__}/__init__.py). Add it there if Metric3D '
        f'has started depending on it.'
    )
