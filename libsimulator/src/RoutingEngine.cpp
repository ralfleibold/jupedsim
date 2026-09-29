// SPDX-License-Identifier: LGPL-3.0-or-later
#include "RoutingEngine.hpp"

#include "SimulationError.hpp"
#include "Visitor.hpp"

#include <CGAL/squared_distance_3.h>

#include <algorithm>
#include <cmath>

RoutingEngine::RoutingEngine(const Geometry& geometry)
    : _geometry(geometry), _shortestPath(geometry)
{
}

DestinationId RoutingEngine::AddDestination(std::span<const DestinationArea> areas)
{
    if(areas.empty()) {
        throw SimulationError("A destination needs at least one area.");
    }
    std::vector<Location> centroids{};
    centroids.reserve(areas.size());
    for(const auto& area : areas) {
        const Point centroid = area.polygon.Centroid();
        const auto face_location =
            _geometry.locate_in_region(area.region, {centroid.x, centroid.y});
        if(face_location.face == SurfaceMesh::null_face()) {
            throw SimulationError(
                "Destination area centroid {} is not inside region {}.", centroid, area.region);
        }
        // Interim until get_location takes a region id: the z on the region's own face pins
        // the location to that face.
        centroids.push_back(
            _geometry.get_location(centroid.x, centroid.y, face_location.point.z(), 0.0).value());
    }
    _destinations.push_back(std::move(centroids));
    return static_cast<DestinationId>(_destinations.size() - 1);
}

Point RoutingEngine::GetOrientation(const Location& from, const RoutingTarget& to)
{
    // Workaround: We will not call a Point2Point engine in the future.
    return std::visit(
        overloaded{
            [&](const Location& place) { return _shortestPath.GetOrientation(from, place); },
            [&](DestinationId id) {
                const auto& destination = _destinations.at(static_cast<std::size_t>(id));
                return _shortestPath.GetOrientation(from, nearest(from, destination));
            }},
        to);
}

const Location&
RoutingEngine::nearest(const Location& from, const std::vector<Location>& destination)
{
    if(destination.size() == 1) {
        return destination.front();
    }
    const auto length = [&](const Location& to) {
        const auto path = _shortestPath.GetShortestPath(from.position_3d(), to.position_3d());
        double sum = 0.0;
        for(std::size_t i = 1; i < path.size(); ++i) {
            sum += std::sqrt(CGAL::to_double(CGAL::squared_distance(path[i - 1], path[i])));
        }
        return sum;
    };
    return *std::ranges::min_element(destination, {}, length);
}
