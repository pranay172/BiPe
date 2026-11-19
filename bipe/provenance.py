"""What a result was run with: the commit and file hashes, for experiment records."""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

from .parameters import ROOT


def commit():
    """``git describe --always --dirty`` of the repository, or None outside git."""
    try:
        return subprocess.run(["git", "-C", str(ROOT), "describe", "--always", "--dirty"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
