// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include "SimulationError.hpp"

#include <cassert>
#include <cstddef>
#include <iterator>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// SurfaceMeshShortestPathRoutingEngine
////////////////////////////////////////////////////////////////////////////////
SurfaceMeshShortestPathRoutingEngine::SurfaceMeshShortestPathRoutingEngine(
    const Geometry3D& geometry)
    : _geometry(geometry)
{
}

bool SurfaceMeshShortestPathRoutingEngine::IsValidLocation(const Location& loc) const
{
    return _geometry.face_below(loc).face != SurfaceMesh::null_face();
}

std::tuple<std::vector<Point3D>, double>
SurfaceMeshShortestPathRoutingEngine::GetShortestPath(const Point3D& source, const Location& target)
{
    const auto from_below = _geometry.face_below(source);
    if(from_below.face == SurfaceMesh::null_face()) {
        throw SimulationError(
            "GetShortestPath(): source does not project onto the walkable surface.");
    }
    const auto target_below = _geometry.face_below(target);
    if(target_below.face == SurfaceMesh::null_face()) {
        throw SimulationError(
            "GetShortestPath(): target does not project onto the walkable surface.");
    }

    // TODO: Implement cache for sequence tree.
    using Traits = CGAL::Surface_mesh_shortest_path_traits<SurfaceKernel, SurfaceMesh>;
    using ShortestPath = CGAL::Surface_mesh_shortest_path<Traits>;
    ShortestPath shortest_path(_geometry.mesh());
    const auto to_loc = shortest_path.locate(target_below.point, _geometry.aabb_tree());
    shortest_path.add_source_point(to_loc);
    shortest_path.build_sequence_tree();

    const auto from_loc = shortest_path.locate(from_below.point, _geometry.aabb_tree());
    std::vector<Point3D> path;
    const auto result = shortest_path.shortest_path_points_to_source_points(
        from_loc.first, from_loc.second, std::back_inserter(path));
    // CGAL returns the cost directly. No separate calculation needed.
    return {std::move(path), result.first};
}

Point SurfaceMeshShortestPathRoutingEngine::GetNextWaypoint(
    const Point3D& source,
    const Location& target)
{
    const auto result = GetShortestPath(source, target);
    const auto& path = std::get<0>(result);
    // Next waypoint, projected onto x/y (z dropped). A query point sitting
    // exactly on a triangle edge makes CGAL emit a duplicate leading waypoint,
    // so skip any waypoints coinciding with the start and return the first one
    // that actually differs.
    for(std::size_t i = 1; i < path.size(); ++i) {
        const Point dir(path[i].x() - path[0].x(), path[i].y() - path[0].y());
        if(!dir.isZeroLength()) {
            return {path[i].x(), path[i].y()};
        }
    }
    // Already at the destination (or degenerate).
    return {source.x(), source.y()};
}

Point SurfaceMeshShortestPathRoutingEngine::GetOrientation(
    const Point3D& source,
    const Location& target)
{
    const Point dir = GetNextWaypoint(source, target) - Point(source.x(), source.y());
    // At the destination -> no direction.
    return dir.isZeroLength() ? Point{0, 0} : dir.Normalized();
}
