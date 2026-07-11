// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Geometry3D.hpp"
#include "RoutingEngine3D.hpp"
#include "SimulationError.hpp"

class TacticalDecisionSystem
{
public:
    TacticalDecisionSystem() = default;
    ~TacticalDecisionSystem() = default;
    TacticalDecisionSystem(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem& operator=(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem(TacticalDecisionSystem&& other) = delete;
    TacticalDecisionSystem& operator=(TacticalDecisionSystem&& other) = delete;

    /// @p surface is the Geometry3D when the simulation runs on the surface
    /// (source height comes from the agent's region anchor), nullptr on the
    /// legacy 2D path (z=0, exact for flat lifts). The target's height is the
    /// stage's anchored z, carried on the agent by the strategical level.
    void Run(RoutingEngine3D& routingEngine, auto&& agents, const Geometry3D* surface) const
    {
        for(auto& agent : agents) {
            const auto dest = agent.target;
            double z = 0.0;
            if(surface != nullptr) {
                const auto anchor =
                    surface->locate_in_region(agent.regionId, {agent.pos.x, agent.pos.y});
                if(anchor.face == SurfaceMesh::null_face()) {
                    throw SimulationError(
                        "Agent {} at {} is not on the surface of region {}",
                        agent.id,
                        agent.pos,
                        agent.regionId);
                }
                z = anchor.point.z();
            }
            agent.destination = routingEngine.GetNextWaypoint(
                {agent.pos.x, agent.pos.y, z}, {dest.x, dest.y, agent.targetZ});
        }
    }
};
