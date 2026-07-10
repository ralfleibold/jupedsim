// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceStep.hpp"

#include <cassert>
#include <cstddef>
#include <utility>

void run_surface_step(
    double dT,
    const OperationalModel& model,
    InformationGatherer3D& gatherer,
    const Geometry3D& geometry,
    AgentContainer<GenericAgent>& agents,
    std::vector<Geometry3D::FaceLocation>& anchors)
{
    assert(agents.size() == anchors.size());
    const auto agentCount = agents.size();

    std::vector<Point3D> positions{};
    positions.reserve(agentCount);
    for(const auto& anchor : anchors) {
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
        const auto from = agents[i].pos;
        model.ApplyUpdate(updates[i], agents[i]);
        anchors[i] = geometry.walk_on_surface(
            geometry.region_of(anchors[i].face),
            {from.x, from.y},
            {agents[i].pos.x, agents[i].pos.y});
    }
}
