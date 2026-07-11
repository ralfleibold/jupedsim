// SPDX-License-Identifier: LGPL-3.0-or-later
#include "InformationGatherer3D.hpp"

#include "Geometry3D.hpp"
#include "SimulationError.hpp"

#include <cassert>
#include <span>
#include <utility>

InformationGatherer3D::InformationGatherer3D(
    const Geometry3D& geometry,
    double cellSize,
    double height)
    : _wallIndex(geometry.mesh()), _search(cellSize), _height(height)
{
}

void InformationGatherer3D::update(
    const AgentContainer<GenericAgent>& agents,
    std::vector<Point3D> positions)
{
    assert(agents.size() == positions.size());
    _agents = &agents;
    _positions = std::move(positions);
    _search.rebuild_index(_positions);
}

void InformationGatherer3D::update(
    const Geometry3D& geometry,
    const AgentContainer<GenericAgent>& agents)
{
    std::vector<Point3D> positions{};
    positions.reserve(agents.size());
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
    update(agents, std::move(positions));
}

void InformationGatherer3D::add(const AgentContainer<GenericAgent>& agents, const Point3D& position)
{
    assert(agents.size() == _positions.size() + 1);
    _agents = &agents;
    _positions.push_back(position);
    _search.add_position(position);
}

InformationForUpdate InformationGatherer3D::gather(
    std::size_t agentIndex,
    const InformationRequirements& requirements,
    std::vector<LineSegment>& wallBuffer) const
{
    return gather_at(_positions[agentIndex], requirements, wallBuffer);
}

InformationForUpdate InformationGatherer3D::gather_at(
    const Point3D& position,
    const InformationRequirements& requirements,
    std::vector<LineSegment>& wallBuffer) const
{
    InformationForUpdate info{};

    if(requirements.neighborRadius) {
        const auto in_horizontal_range =
            within_horizontal_distance(position, *requirements.neighborRadius);
        const auto in_vertical_band = within_vertical_band(position, _height);
        for(const auto candidate :
            _search.candidates(position, *requirements.neighborRadius, _height)) {
            if(in_horizontal_range(_positions[candidate]) &&
               in_vertical_band(_positions[candidate])) {
                info.neighbors.push_back((*_agents)[candidate]);
            }
        }
    }

    wallBuffer.clear();
    if(requirements.wallRadius) {
        for(const auto& wall :
            _wallIndex.get_near_walls(position, *requirements.wallRadius, _height)) {
            wallBuffer.emplace_back(
                Point{wall.source().x(), wall.source().y()},
                Point{wall.target().x(), wall.target().y()});
        }
        info.walls = std::span<const LineSegment>(wallBuffer.data(), wallBuffer.size());
    }
    return info;
}
