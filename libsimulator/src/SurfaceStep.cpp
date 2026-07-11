// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceStep.hpp"

#include <cstddef>
#include <utility>
#include <vector>

void run_surface_step(
    double dT,
    const OperationalModel& model,
    InformationGatherer3D& gatherer,
    const Geometry3D& geometry,
    AgentContainer<GenericAgent>& agents)
{
    const auto agentCount = agents.size();

    // Compute all updates from the frozen pre-step state (the caller updated
    // the gatherer for this iteration). The wall buffer is reused across
    // agents: info is consumed within the model call.
    std::vector<OperationalModelUpdate> updates{};
    updates.reserve(agentCount);
    std::vector<LineSegment> wallBuffer{};
    for(std::size_t i = 0; i < agentCount; ++i) {
        const auto info = gatherer.gather(i, model.Requirements(agents[i]), wallBuffer);
        updates.push_back(model.ComputeNewPosition(dT, agents[i], info));
    }

    // Apply, re-anchor the walked 2D step onto the surface, and re-index so
    // queries between steps (public API, agent validation) see the post-step
    // anchors.
    std::vector<Point3D> anchors{};
    anchors.reserve(agentCount);
    for(std::size_t i = 0; i < agentCount; ++i) {
        auto& agent = agents[i];
        const auto from = agent.pos;
        model.ApplyUpdate(updates[i], agent);
        const auto anchor =
            geometry.walk_on_surface(agent.regionId, {from.x, from.y}, {agent.pos.x, agent.pos.y});
        agent.regionId = geometry.region_of(anchor.face);
        agent.z = anchor.point.z();
        anchors.push_back(anchor.point);
    }
    gatherer.update(agents, std::move(anchors));
}
