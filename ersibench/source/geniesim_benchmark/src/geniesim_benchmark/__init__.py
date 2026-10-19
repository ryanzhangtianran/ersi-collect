# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""geniesim_benchmark — benchmark distribution for the geniesim meta-package.

Holds every subpackage that previously lived under ``geniesim.*``:

    geniesim_benchmark.app          # robot controllers, ros publishers, workflow
    geniesim_benchmark.benchmark    # task benchmark, envs, hooks, policies, tasks
    geniesim_benchmark.config       # task / robust / spatial / s2r yaml configs
    geniesim_benchmark.evaluator    # llm-based scene & instruction generators
    geniesim_benchmark.plugins      # ader, logger, output_system, tgs, sim_control_gui
    geniesim_benchmark.robot        # robot urdf assets and helpers
    geniesim_benchmark.utils        # ros nodes, comm, transform / name / data utils

Installed via ``pip install geniesim[benchmark]`` (meta-package extra).
"""

from geniesim_benchmark._version import _resolve_version

__version__: str = _resolve_version("geniesim_benchmark")

__all__ = ["__version__"]
