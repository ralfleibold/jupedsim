// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "GenericAgent.hpp"
#include "InformationForUpdate.hpp"
#include "NeighborhoodSearch3D.hpp"
#include "WallIndex.hpp"

#include <cstddef>
#include <vector>

class Geometry3D;

/// Fills InformationForUpdate for agents living on a Geometry3D surface: the
/// 3D counterpart of the gather step in OperationalDecisionSystem. Neighbors
/// are selected by horizontal distance plus a vertical band, so stacked
/// floors separate while region seams do not (region ids play no role here).
/// Walls are the border edges of the surface, z-scoped the same way and
/// projected to 2D for the models.
class InformationGatherer3D
{
    WallIndex _wallIndex;
    NeighborhoodSearch3D _search;
    double _height;
    const AgentContainer<GenericAgent>* _agents{nullptr};
    std::vector<Point3D> _positions{};

public:
    /// @param geometry the surface the agents walk on.
    /// @param cellSize broad-phase grid cell size of the neighborhood search.
    /// @param height vertical band: how far apart in z two things may be and
    ///        still interact (agent-agent and agent-wall alike).
    InformationGatherer3D(const Geometry3D& geometry, double cellSize, double height);

    /// Re-index for this step. @p positions are the agents' on-surface anchor
    /// points, one per agent in @p agents enumeration order. @p agents must
    /// outlive the gather calls of this step.
    void update(const AgentContainer<GenericAgent>& agents, std::vector<Point3D> positions);

    /// Information for the agent at @p agentIndex (enumeration order of
    /// update()). @p wallBuffer receives the projected wall segments; the
    /// returned info.walls points into it, so it must stay alive and
    /// unmodified for as long as the info is in use.
    InformationForUpdate gather(
        std::size_t agentIndex,
        const InformationRequirements& requirements,
        std::vector<LineSegment>& wallBuffer) const;
};
