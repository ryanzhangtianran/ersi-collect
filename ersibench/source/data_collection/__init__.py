# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Locator package for the ``data_collection`` module.

This package exists so ``geniesim_cli`` can resolve the module's on-disk
location via ``importlib.util.find_spec("data_collection")`` — parity with how
``geniesim_benchmark`` is discovered. It is a thin, reversible shim: the runtime
code under ``client/`` ``server/`` ``common/`` runs as scripts with this
directory as cwd and is intentionally **not** exposed as importable subpackages
here. Removing this file + ``pyproject.toml`` (and ``pip uninstall
geniesim-data-collection``) fully reverts to filesystem-walk discovery.
"""

__version__ = "0.0.0"
