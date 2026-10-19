"""Bounded direct-optimization then graph-seeded planning."""

import logging
import time


def plan_with_graph_fallback(motion_gen, start, goal, config, link_poses=None, allow_graph=True):
    """Keep both attempts within one configured timeout (cuRobo checks between iterations)."""
    started = time.monotonic()
    direct = config.clone()
    direct.enable_graph = False
    direct.need_graph_success = False
    direct.enable_graph_attempt = None
    direct.timeout = config.timeout * (0.7 if allow_graph else 1.0)
    result = motion_gen.plan_batch(start, goal, direct, link_poses=link_poses)
    status = getattr(result.status, "name", str(result.status))
    remaining = config.timeout - (time.monotonic() - started)
    if (not allow_graph or result.success.any() or not result.valid_query or remaining <= 0
            or status not in ("TRAJOPT_FAIL", "FINETUNE_TRAJOPT_FAIL")):
        return result
    graph = config.clone()
    graph.enable_graph = True
    graph.enable_graph_attempt = None
    graph.need_graph_success = False
    graph.num_graph_seeds = 1
    graph.num_trajopt_seeds = 1
    graph.max_attempts = min(config.max_attempts, 4)
    graph.timeout = remaining
    logging.getLogger(__name__).info("GRAPH_FALLBACK reason=%s budget_s=%.2f", status, remaining)
    return motion_gen.plan_batch(start, goal, graph, link_poses=link_poses)
