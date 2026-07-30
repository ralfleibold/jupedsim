#!/usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later
"""What it costs a Python model to delegate a step to a built-in model.

All timed runs compute the identical physics (CollisionFreeSpeedModelV3), so the
difference to the native run is pure overhead of the delegation machinery.

The overhead has two parts, and they scale differently: one call per agent and
step for the delegation itself, one call per *neighbor* for the state mapping.
Run this at two densities to tell them apart -- with

    overhead = per_agent * agents + per_neighbor * mapping_calls

two runs give you both terms.

Usage (from the repo root)::

    PYTHONPATH=python_modules/jupedsim:build/lib \\
        python examples/benchmark_model_delegation.py
    PYTHONPATH=python_modules/jupedsim:build/lib \\
        python examples/benchmark_model_delegation.py --spacing 1.4
"""

import argparse
import time
from dataclasses import dataclass, replace

import jupedsim as jps
import shapely


@dataclass(kw_only=True, frozen=True)
class Wrapped:
    """Custom state carrying a built-in model state as its payload."""

    sub: object


class Delegating(jps.CustomOperationalModel):
    """Delegates every step, mapping neighbors 1:1 to the built-in state."""

    def __init__(self):
        super().__init__()
        self._inner = jps.CollisionFreeSpeedModelV3()
        self.mapping_calls = 0

    def _as_inner_state(self, neighbor):
        self.mapping_calls += 1
        return neighbor.sub

    def compute_next_state(self, state, step):
        step = step.with_neighbor_states(self._as_inner_state)
        sub, movement = self._inner.compute_next_state(state.sub, step)
        return replace(state, sub=sub), movement


class PurePython(jps.CustomOperationalModel):
    """Reference point: a hand-written model with its own neighbor loop.

    Deliberately much simpler than CollisionFreeSpeedModelV3 -- it is a floor for
    what reimplementing in Python costs, not a fair reimplementation.
    """

    def compute_next_state(self, state, step):
        dx, dy = step.to_next_target
        norm = (dx * dx + dy * dy) ** 0.5 or 1.0
        spacing = 100.0
        for neighbor in step.other_agents_in_range(3.0):
            rx, ry = neighbor.relative_position
            if rx * dx + ry * dy > 0.0:
                spacing = min(spacing, (rx * rx + ry * ry) ** 0.5)
        speed = min(1.2, max(0.0, spacing - 0.4))
        return replace(state, sub=state.sub), (
            dx / norm * speed * step.dt,
            dy / norm * speed * step.dt,
        )


def build(model, state_of, *, rows, per_row, spacing):
    sim = jps.Simulation(
        model=model,
        geometry=shapely.Polygon([(0, 0), (60, 0), (60, 30), (0, 30)]),
        dt=0.05,
    )
    exit_id = sim.add_exit_stage([(59, 14), (59, 16), (60, 16), (60, 14)])
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    for row in range(rows):
        for col in range(per_row):
            sim.add_agent(
                journey_id=journey_id,
                stage_id=exit_id,
                position=(1.0 + col * spacing, 1.0 + row * spacing),
                state=state_of(),
            )
    return sim


def timed(label, sim, steps):
    start = time.perf_counter()
    for _ in range(steps):
        sim.iterate()
    elapsed = time.perf_counter() - start
    print(
        f"{label:<34} {elapsed:7.3f} s   {elapsed / steps * 1000:6.2f} ms/step"
    )
    return elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=20)
    parser.add_argument("--per-row", type=int, default=20)
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.8,
        help="grid spacing in m; smaller means more neighbors per agent",
    )
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args()

    agents = args.rows * args.per_row
    layout = {
        "rows": args.rows,
        "per_row": args.per_row,
        "spacing": args.spacing,
    }
    print(f"{agents} agents, {args.steps} steps, spacing {args.spacing} m\n")

    native = timed(
        "native CFSM V3 (C++ only)",
        build(
            jps.CollisionFreeSpeedModelV3(),
            jps.CollisionFreeSpeedModelV3State,
            **layout,
        ),
        args.steps,
    )
    delegating = Delegating()
    delegated = timed(
        "delegated + mapped neighbors",
        build(
            delegating,
            lambda: Wrapped(sub=jps.CollisionFreeSpeedModelV3State()),
            **layout,
        ),
        args.steps,
    )
    pure = timed(
        "pure Python model (neighbor loop)",
        build(
            PurePython(),
            lambda: Wrapped(sub=jps.CollisionFreeSpeedModelV3State()),
            **layout,
        ),
        args.steps,
    )

    calls = delegating.mapping_calls
    overhead = delegated - native
    print()
    print(f"delegated                : {delegated / native:5.1f}x native")
    print(f"pure Python model        : {pure / native:5.1f}x native")
    print(
        f"mapping calls            : {calls} "
        f"({calls / args.steps:.0f}/step, "
        f"{calls / args.steps / agents:.1f} per agent-step)"
    )
    print(
        f"overhead                 : {overhead / args.steps * 1000:6.2f} ms/step "
        f"= per_agent * {agents} + per_neighbor * {calls / args.steps:.0f}"
    )


if __name__ == "__main__":
    main()
