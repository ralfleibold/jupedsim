// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "RoutingEngine3D.hpp"

class TacticalDecisionSystem
{
public:
    TacticalDecisionSystem() = default;
    ~TacticalDecisionSystem() = default;
    TacticalDecisionSystem(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem& operator=(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem(TacticalDecisionSystem&& other) = delete;
    TacticalDecisionSystem& operator=(TacticalDecisionSystem&& other) = delete;

    void Run(RoutingEngine3D& routingEngine, auto&& agents) const
    {
        // z=0 is transitional: agents carry no height yet; after the switch
        // the on-surface anchor supplies it.
        for(auto& agent : agents) {
            const auto dest = agent.target;
            agent.destination = routingEngine.GetNextWaypoint(
                {agent.pos.x, agent.pos.y, 0.0}, {dest.x, dest.y, 0.0});
        }
    }
};
