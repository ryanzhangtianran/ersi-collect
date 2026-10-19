# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Resolve the package version from the repo-root ``VERSION`` file.

See ``geniesim/src/geniesim/_version.py`` for the full design note;
this file is the same logic duplicated for the ``geniesim_benchmark``
distribution (each peer must be self-contained, no cross-imports).
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version as _pkg_version
from pathlib import Path


def _read_repo_version() -> str | None:
    here = Path(__file__).resolve()
    for ancestor in (here.parents[4] if len(here.parents) > 4 else here.parent, *here.parents):
        candidate = ancestor / "VERSION"
        if candidate.is_file():
            text = candidate.read_text().strip()
            if text:
                return text
    return None


def _resolve_version(dist_name: str) -> str:
    repo = _read_repo_version()
    if repo:
        return repo
    try:
        return _pkg_version(dist_name)
    except PackageNotFoundError:
        return "0.0.0+unknown"
