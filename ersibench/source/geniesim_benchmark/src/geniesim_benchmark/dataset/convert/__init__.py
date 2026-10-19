# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Dataset format converters (agibot ↔ LeRobot, …).

Each converter ships a public ``convert_<from>_to_<to>()`` Python API
plus a ``convert_cli(argv)`` wrapper that the
``geniesim dataset convert <from-to>`` CLI dispatcher calls into. The
CLI wrapper is the only place ``argparse`` lives — the API is
plain-Python so the converter is usable from notebooks and pipelines.
"""

from geniesim_benchmark.dataset.convert.agibot_to_lerobot import (
    convert_agibot_to_lerobot,
)

__all__ = ["convert_agibot_to_lerobot"]
