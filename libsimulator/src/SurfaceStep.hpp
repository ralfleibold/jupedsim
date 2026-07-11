// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "InformationGatherer3D.hpp"
#include "OperationalModel.hpp"

/// One operational step for agents anchored on a Geometry3D surface:
/// gather (3D neighbors + walls) -> compute (unchanged 2D models) ->
/// apply -> re-anchor each agent's 2D position onto the surface via
/// walk_on_surface (new z, possibly flipped region).
///
/// Requires @p gatherer to be updated for this iteration (the caller anchors
/// the agents via gatherer.update(geometry, agents) after agent removal --
/// the surface counterpart of NeighborhoodSearch::Update). On return the
/// gatherer is re-indexed on the post-step anchors.
///
/// Each agent's on-surface anchor is (agent.pos, agent.regionId); the region
/// id selects the sheet where floors stack in (x,y). walk_on_surface updates
/// it as agents cross region seams.
///
/// Transitional composition: once the 3D pipeline reproduces the 2D results,
/// this replaces the world-specific parts of OperationalDecisionSystem::Run --
/// there will be one simulation, not a parallel 3D one.
void run_surface_step(
    double dT,
    const OperationalModel& model,
    InformationGatherer3D& gatherer,
    const Geometry3D& geometry,
    AgentContainer<GenericAgent>& agents);
