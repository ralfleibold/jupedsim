# SPDX-License-Identifier: LGPL-3.0-or-later
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import (
    TYPE_CHECKING,
    Protocol,
    runtime_checkable,
)

if TYPE_CHECKING:
    from jupedsim.agent import Agent
    from jupedsim.linesegment import LineSegment


@runtime_checkable
class CustomModelAgentState(Protocol):
    """Structural interface for per-agent model state of custom models.

    Any object exposing a ``position`` attribute of type ``tuple[float, float]``
    satisfies this protocol -- explicitly subclassing it is supported but not
    required. The runtime check performed when adding an agent verifies
    attribute presence only; value types are validated by the simulation
    itself.
    """

    position: tuple[float, float]


@dataclass(kw_only=True)
class CustomModelAgentParameters:
    """Parameters required to create an agent for a custom model.

    ``model`` is the agent's initial per-agent model state and is **required**:
    you must set it to your own object satisfying :class:`CustomModelAgentState`
    -- by subclassing it or simply by exposing a ``position`` attribute -- which
    carries the agent's ``position``, from which the agent is spawned. It should be an immutable object -- a ``@dataclass(frozen=True)`` is
    strongly recommended -- because the simulation shares it live with your model
    during each step (see :class:`CustomOperationalModel`). ``model`` has no
    default; omitting it raises a ``TypeError`` at construction.
    """

    journey_id: int = 0
    stage_id: int = 0
    model: CustomModelAgentState


@dataclass(frozen=True)
class InformationRequirements:
    """What the simulation should gather for one agent before calling into the
    model (:meth:`CustomOperationalModel.compute_new_position` or
    :meth:`CustomOperationalModel.check_model_constraint`).

    ``None`` means "not needed, do not compute". Requirements are queried anew
    for every agent in every step, so they may vary between agents and over
    time.
    """

    neighbor_radius: float | None = None
    """Gather all agents within this radius [m] of the agent's position."""

    wall_radius: float | None = None
    """Gather all wall segments within this radius [m] of the agent's
    position. The delivered set may be over-inclusive; apply exact distances
    yourself."""


@dataclass(frozen=True)
class InformationForUpdate:
    """Per-agent input gathered by the simulation according to the model's
    :class:`InformationRequirements`.

    A model is a pure function of ``(dt, ped, info)`` -- it never queries the
    simulation itself. Fields not requested are empty lists.
    """

    neighbors: list[Agent] = field(default_factory=list)
    """All agents within ``neighbor_radius`` of the agent's position, the
    agent itself included."""

    walls: list[LineSegment] = field(default_factory=list)
    """All wall segments within ``wall_radius`` of the agent's position.
    May be over-inclusive (segments further away can appear)."""


class CustomOperationalModel(ABC):
    """Base class for operational models implemented in Python.

    Subclasses implement :meth:`compute_new_position` and optionally
    :meth:`information_requirements`, :meth:`check_model_constraint` and
    :meth:`constraint_requirements`. Constraint violations should be reported
    by raising an exception.

    Before each call into the model, the simulation asks
    :meth:`information_requirements` (resp. :meth:`constraint_requirements`)
    what to gather for the agent and passes the result as
    :class:`InformationForUpdate`. The default implementations request
    nothing -- override them to receive neighbors and walls.

    .. warning::

        **Per-agent model state is live and shared -- never mutate it in place.**

        The ``ped.model`` object you receive (and every neighbor's ``.model``
        in :class:`InformationForUpdate`) is the agent's *live* state, shared
        by reference with the running simulation for performance. JuPedSim
        advances agents in two phases per step: it first *computes* every
        agent's update from the current state of all agents, then *applies* all
        updates together. Mutating ``ped.model`` (or a neighbor's) during the
        compute phase changes state that other agents are still reading in the
        same step, silently breaking the compute-then-apply ordering and
        producing order-dependent results.

        The only correct way to change state is to return a new state object
        from :meth:`compute_new_position` -- returning ``ped.model`` itself
        (even unchanged) raises an error; use
        ``dataclasses.replace(ped.model, ...)``. Make your state type
        immutable -- a ``@dataclass(frozen=True)`` -- so accidental in-place
        writes raise immediately instead of silently corrupting the
        simulation.
    """

    def information_requirements(self, ped: Agent) -> InformationRequirements:
        """What to gather for ``ped`` before :meth:`compute_new_position`."""
        return InformationRequirements()

    def constraint_requirements(self, ped: Agent) -> InformationRequirements:
        """What to gather for ``ped`` before :meth:`check_model_constraint`."""
        return InformationRequirements()

    @abstractmethod
    def compute_new_position(
        self,
        dt: float,
        ped: Agent,
        info: InformationForUpdate,
    ) -> CustomModelAgentState:
        """Compute one update for ``ped``."""

    def check_model_constraint(
        self,
        ped: Agent,
        info: InformationForUpdate,
    ) -> None:
        """Raise an exception when ``ped`` violates this model's constraints."""
        pass

    def _information_requirements(self, ped):
        from jupedsim.agent import Agent

        requirements = self.information_requirements(Agent(ped))
        return (requirements.neighbor_radius, requirements.wall_radius)

    def _constraint_requirements(self, ped):
        from jupedsim.agent import Agent

        requirements = self.constraint_requirements(Agent(ped))
        return (requirements.neighbor_radius, requirements.wall_radius)

    def _compute_new_position(
        self,
        dt,
        ped,
        neighbors,
        walls,
    ) -> CustomModelAgentState:
        from jupedsim.agent import Agent
        from jupedsim.linesegment import LineSegment

        return self.compute_new_position(
            dt,
            Agent(ped),
            InformationForUpdate(
                neighbors=[Agent(neighbor) for neighbor in neighbors],
                walls=[LineSegment(wall) for wall in walls],
            ),
        )

    def _check_model_constraint(
        self,
        ped,
        neighbors,
        walls,
    ) -> None:
        from jupedsim.agent import Agent
        from jupedsim.linesegment import LineSegment

        self.check_model_constraint(
            Agent(ped),
            InformationForUpdate(
                neighbors=[Agent(neighbor) for neighbor in neighbors],
                walls=[LineSegment(wall) for wall in walls],
            ),
        )
