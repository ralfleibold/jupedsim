// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "Mesh.hpp"
#include "Point.hpp"
#include "RoutingEngine.hpp"

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Exact geodesic any-angle routing via CGAL's Surface_mesh_shortest_path
// (MMP / continuous-Dijkstra). The walkable region is lifted from the global
// CDT into a flat (z = 0) EPICK surface mesh; geodesics on that surface route
// around holes and equal the 2D Euclidean shortest path.
//
// EPICK is used locally (not the global Simple_cartesian kernel K) because the
// MMP window propagation needs robust exact predicates; the lift is a trivial
// double -> double copy with z = 0.
class SurfaceMeshShortestPathRoutingEngine : public RoutingEngine
{
public:
    using SK = CGAL::Exact_predicates_inexact_constructions_kernel;
    using SurfaceMesh = CGAL::Surface_mesh<SK::Point_3>;
    using ShortestPathTraits = CGAL::Surface_mesh_shortest_path_traits<SK, SurfaceMesh>;
    using ShortestPath = CGAL::Surface_mesh_shortest_path<ShortestPathTraits>;
    using AABBPrimitive = CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh>;
    using AABBTraits = CGAL::AABB_traits_3<SK, AABBPrimitive>;
    using AABBTree = CGAL::AABB_tree<AABBTraits>;

private:
    CDT cdt{};
    std::unique_ptr<Mesh> mesh{};
    SurfaceMesh _surfaceMesh{};
    AABBTree _aabbTree{};

public:
    SurfaceMeshShortestPathRoutingEngine() = default;
    ~SurfaceMeshShortestPathRoutingEngine() override = default;

    SurfaceMeshShortestPathRoutingEngine(const SurfaceMeshShortestPathRoutingEngine&) = delete;
    SurfaceMeshShortestPathRoutingEngine&
    operator=(const SurfaceMeshShortestPathRoutingEngine&) = delete;

    SurfaceMeshShortestPathRoutingEngine(SurfaceMeshShortestPathRoutingEngine&&) = default;
    SurfaceMeshShortestPathRoutingEngine&
    operator=(SurfaceMeshShortestPathRoutingEngine&&) = default;

    std::string name() const override { return "SurfaceMeshShortestPath"; }
    void set_geometry(const CollisionGeometry& geometry) override;
    std::vector<Point> compute_waypoints(Point from, Point destination) override;
    bool is_routable(Point p) const override;

    const Mesh* MeshData() const { return mesh.get(); }

private:
    CDT::Face_handle find_face(K::Point_2 p) const;
    void buildSurfaceMesh();
};
