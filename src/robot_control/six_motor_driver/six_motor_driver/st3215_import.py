"""Load the ST3215 library from the installed package or legacy checkout."""

from __future__ import annotations

import sys
from pathlib import Path


def load_st3215():
    try:
        from st3215 import ST3215
    except ImportError:
        legacy_checkout = Path.home() / "libs" / "python-st3215"
        if not legacy_checkout.is_dir():
            raise ImportError(
                "The Python package 'st3215' is required for arm hardware. "
                "Run ./tools/setup-linux --hardware."
            ) from None
        sys.path.insert(0, str(legacy_checkout))
        from st3215 import ST3215
    return ST3215
