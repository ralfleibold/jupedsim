// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include "CfgCgal.hpp"
#include "CollisionGeometry.hpp"
#include "Mesh.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"

#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/number_utils.h>

#include <cmath>
#include <cstddef>
#include <iterator>
#include <unordered_map>
#include <vector>

namespace
{
struct VertexHandleHash {
    std::size_t operator()(CDT::Vertex_handle vh) const noexcept
    {
        // Vertex_handle is an iterator; hash the pointed-to address (see FaceHandleHash in
        // AStarRoutingEngine).
        return std::hash<decltype(&*vh)>{}(&*vh);
    }
};

// The MMP geodesic is already taut, but shortest_path_points reports a point wherever the path
// crosses a CDT diagonal -- including crossings that fall on a straight run (not a turn). Drop
// those so the output is corner-only, matching the funnel result of AStarRoutingEngine.
std::vector<Point> strip_collinear(const std::vector<Point>& pts)
{
    if(pts.size() <= 2) {
        return pts;
    }
    std::vector<Point> out{};
    out.reserve(pts.size());
    out.push_back(pts.front());
    for(std::size_t i = 1; i + 1 < pts.size(); ++i) {
        const auto& a = out.back(); // last kept vertex
        const auto& b = pts[i];
        const auto& c = pts[i + 1];
        const double abx = b.x - a.x, aby = b.y - a.y;
        const double acx = c.x - a.x, acy = c.y - a.y;
        const double cross = abx * acy - aby * acx; // 2 * area(a, b, c)
        const double ac_len = std::hypot(acx, acy);
        // Threshold is a perpendicular distance in metres, so it is independent of segment
        // length. EPICK has inexact constructions, so crossing points are only collinear up to
        // floating error -- never bit-exact.
        if(ac_len < 1e-12 || std::abs(cross) / ac_len > 1e-7) {
            out.push_back(b);
        }
    }
    out.push_back(pts.back());
    return out;
}
} // namespace

void SurfaceMeshShortestPathRoutingEngine::set_geometry(const CollisionGeometry& geometry)
{
    const auto& poly = geometry.Polygon();
    cdt = CDT{};
    cdt.insert_constraint(
        poly.outer_boundary().vertices_begin(), poly.outer_boundary().vertices_end(), true);
    for(const auto& hole : poly.holes()) {
        cdt.insert_constraint(hole.vertices_begin(), hole.vertices_end(), true);
    }
    CGAL::mark_domain_in_triangulation(cdt);
    mesh = std::make_unique<Mesh>(cdt);
    buildSurfaceMesh();
}

void SurfaceMeshShortestPathRoutingEngine::buildSurfaceMesh()
{
    _surfaceMesh.clear();
    std::unordered_map<CDT::Vertex_handle, SurfaceMesh::Vertex_index, VertexHandleHash> vertexMap{};

    const auto vertex_for = [&](CDT::Vertex_handle vh) {
        if(const auto it = vertexMap.find(vh); it != vertexMap.end()) {
            return it->second;
        }
        const auto& p = vh->point();
        const auto idx = _surfaceMesh.add_vertex(
            SK::Point_3(CGAL::to_double(p.x()), CGAL::to_double(p.y()), 0.0));
        vertexMap.emplace(vh, idx);
        return idx;
    };

    // CDT in-domain faces are CCW oriented; emitting vertices in CDT order gives every surface
    // face a consistent +z normal, so shared edges are traversed in opposite directions and the
    // result is a valid manifold-with-boundary triangle mesh.
    for(auto fh = cdt.finite_faces_begin(); fh != cdt.finite_faces_end(); ++fh) {
        if(!fh->get_in_domain()) {
            continue;
        }
        _surfaceMesh.add_face(
            vertex_for(fh->vertex(0)), vertex_for(fh->vertex(1)), vertex_for(fh->vertex(2)));
    }

    _aabbTree.clear();
    ShortestPath(_surfaceMesh).build_aabb_tree(_aabbTree);
}

std::vector<Point>
SurfaceMeshShortestPathRoutingEngine::compute_waypoints(Point from, Point destination)
{
    if(!mesh) {
        throw SimulationError(
            "SurfaceMeshShortestPathRoutingEngine has no geometry; call set_geometry first");
    }
    // Validate against the CDT for clean error parity with AStarRoutingEngine; the surface-mesh
    // locate() would otherwise silently project an out-of-domain point onto the nearest face.
    find_face({from.x, from.y});
    find_face({destination.x, destination.y});

    ShortestPath shortest_path(_surfaceMesh);

    // Geodesic field is built FROM the source. Seed the destination as the single source and query
    // the start point, so the returned path runs start -> destination.
    const auto destination_loc =
        shortest_path.locate(SK::Point_3(destination.x, destination.y, 0.0), _aabbTree);
    shortest_path.add_source_point(destination_loc);

    const auto from_loc = shortest_path.locate(SK::Point_3(from.x, from.y, 0.0), _aabbTree);

    std::vector<SK::Point_3> points{};
    shortest_path.shortest_path_points_to_source_points(
        from_loc.first, from_loc.second, std::back_inserter(points));

    if(points.empty()) {
        return std::vector<Point>{from, destination};
    }

    std::vector<Point> waypoints{};
    waypoints.reserve(points.size());
    for(const auto& p : points) {
        waypoints.emplace_back(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    }
    // The algorithm also creates intermediate points on edges even if part of a "straight line".
    // Stripping those...
    return strip_collinear(waypoints);
}

bool SurfaceMeshShortestPathRoutingEngine::is_routable(Point p) const
{
    if(!mesh) {
        throw SimulationError(
            "SurfaceMeshShortestPathRoutingEngine has no geometry; call set_geometry first");
    }
    try {
        find_face({p.x, p.y});
    } catch(const SimulationError&) {
        return false;
    }
    return true;
}

CDT::Face_handle SurfaceMeshShortestPathRoutingEngine::find_face(K::Point_2 p) const
{
    const auto face = cdt.locate(p);
    if(face == nullptr || cdt.is_infinite(face) || !face->get_in_domain()) {
        throw SimulationError(
            "Point ({}, {}) is outside of accessible area",
            CGAL::to_double(p.x()),
            CGAL::to_double(p.y()));
    }
    return face;
}
