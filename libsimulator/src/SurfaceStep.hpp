// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "InformationGatherer3D.hpp"
#include "OperationalModel.hpp"

#include <vector>

/// One operational step for agents anchored on a Geometry3D surface:
/// gather (3D neighbors + walls) -> compute (unchanged 2D models) ->
/// apply -> re-anchor each agent's 2D position onto the surface via
/// walk_on_surface (new z, possibly flipped region).
///
/// @p anchors holds each agent's on-surface location, one per agent in
/// @p agents enumeration order; it is advanced alongside the agents.
///
/// Transitional composition: once the 3D pipeline reproduces the 2D results,
/// this replaces the world-specific parts of OperationalDecisionSystem::Run --
/// there will be one simulation, not a parallel 3D one.
void run_surface_step(
    double dT,
    const OperationalModel& model,
    InformationGatherer3D& gatherer,
    const Geometry3D& geometry,
    AgentContainer<GenericAgent>& agents,
    std::vector<Geometry3D::FaceLocation>& anchors);
