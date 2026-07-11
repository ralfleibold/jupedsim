// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceStep.hpp"

#include "SimulationError.hpp"

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

    std::vector<Point3D> positions{};
    positions.reserve(agentCount);
    for(const auto& agent : agents) {
        const auto anchor = geometry.locate_in_region(agent.regionId, {agent.pos.x, agent.pos.y});
        if(anchor.face == SurfaceMesh::null_face()) {
            throw SimulationError(
                "Agent {} at {} is not on the surface of region {}",
                agent.id,
                agent.pos,
                agent.regionId);
        }
        positions.push_back(anchor.point);
    }
    gatherer.update(agents, std::move(positions));

    // Compute all updates from the frozen pre-step state. The wall buffer is
    // reused across agents: info is consumed within the model call.
    std::vector<OperationalModelUpdate> updates{};
    updates.reserve(agentCount);
    std::vector<LineSegment> wallBuffer{};
    for(std::size_t i = 0; i < agentCount; ++i) {
        const auto info = gatherer.gather(i, model.Requirements(agents[i]), wallBuffer);
        updates.push_back(model.ComputeNewPosition(dT, agents[i], info));
    }

    // Apply, then re-anchor the walked 2D step onto the surface.
    for(std::size_t i = 0; i < agentCount; ++i) {
        auto& agent = agents[i];
        const auto from = agent.pos;
        model.ApplyUpdate(updates[i], agent);
        const auto anchor =
            geometry.walk_on_surface(agent.regionId, {from.x, from.y}, {agent.pos.x, agent.pos.y});
        agent.regionId = geometry.region_of(anchor.face);
    }
}
