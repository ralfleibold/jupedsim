// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CfgCgal.hpp"
#include "Geometry3D.hpp"
#include "RoutingEngine.hpp"
#include "SimulationError.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include <CGAL/mark_domain_in_triangulation.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <span>
#include <vector>

namespace
{
/// A single flat 10x10 square at z=0, split into two triangles (CCW).
SurfaceMesh unit_square_mesh()
{
    SurfaceMesh mesh{};
    const auto a = mesh.add_vertex({0, 0, 0});
    const auto b = mesh.add_vertex({10, 0, 0});
    const auto c = mesh.add_vertex({10, 10, 0});
    const auto d = mesh.add_vertex({0, 10, 0});
    mesh.add_face(a, b, c);
    mesh.add_face(a, c, d);
    return mesh;
}

/// A flat floor (z=0, y in [0,10]) joined at the seam y=10 to a 45-degree ramp
/// (z = y-10, y in [10,15]). The ramp is tilted, not vertical, so face_below's
/// -z ray projects onto it cleanly.
SurfaceMesh folded_mesh()
{
    SurfaceMesh mesh{};
    const auto v0 = mesh.add_vertex({0, 0, 0});
    const auto v1 = mesh.add_vertex({10, 0, 0});
    const auto v2 = mesh.add_vertex({10, 10, 0}); // seam
    const auto v3 = mesh.add_vertex({0, 10, 0}); // seam
    const auto v4 = mesh.add_vertex({10, 15, 5}); // ramp top
    const auto v5 = mesh.add_vertex({0, 15, 5}); // ramp top
    mesh.add_face(v0, v1, v2);
    mesh.add_face(v0, v2, v3);
    mesh.add_face(v3, v2, v4);
    mesh.add_face(v3, v4, v5);
    return mesh;
}

/// Build a SurfaceMesh from a 2D polygon by constrained-Delaunay triangulating it.
/// Default height => flat z=0.
SurfaceMesh mesh_from_polygon(const std::vector<K::Point_2>& outer, double z = 0.0)
{
    // Inspired by https://doc.cgal.org/latest/Mesh_2, example Mesh_2/mesh_marked_domain.cpp
    CDT cdt{};
    cdt.insert_constraint(outer.begin(), outer.end(), true); // true => closed
    CGAL::mark_domain_in_triangulation(cdt);

    SurfaceMesh mesh{};
    std::map<CDT::Vertex_handle, SurfaceMesh::Vertex_index> idx;
    const auto vertex_of = [&](CDT::Vertex_handle v) {
        const auto it = idx.find(v);
        if(it != idx.end()) {
            return it->second;
        }
        const auto p = v->point();
        return idx[v] = mesh.add_vertex({p.x(), p.y(), z});
    };
    for(auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
        if(f->get_in_domain()) {
            mesh.add_face(
                vertex_of(f->vertex(0)), vertex_of(f->vertex(1)), vertex_of(f->vertex(2)));
        }
    }
    return mesh;
}

/// All points collinear (in xy) with the segment points.front()->points.back():
/// i.e. the (sub)path is a straight line in the plane. Takes a span so callers
/// can pass a slice of a path (e.g. the part before/after a seam).
::testing::AssertionResult PointsCollinearXY(std::span<const Point3D> points)
{
    const Point d(points.back().x() - points.front().x(), points.back().y() - points.front().y());
    for(const auto& p : points) {
        const Point v(p.x() - points.front().x(), p.y() - points.front().y());
        const double cross = d.CrossProduct(v);
        if(std::abs(cross) > 1e-6) {
            return ::testing::AssertionFailure() << "point (" << p.x() << ", " << p.y()
                                                 << ") off the line (cross=" << cross << ")";
        }
    }
    return ::testing::AssertionSuccess();
}
} // namespace

class FlatSquare : public ::testing::Test
{
public:
    void SetUp() override
    {
        geometry.initialize_from_mesh(unit_square_mesh());
        engine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(geometry);
    }

protected:
    Geometry3D geometry{};
    std::unique_ptr<SurfaceMeshShortestPathRoutingEngine> engine{};
};

TEST_F(FlatSquare, PointAboveSurfaceIsValid)
{
    // z above the surface: the -z ray of face_below projects down onto z=0.
    EXPECT_TRUE(engine->IsValidLocation({5, 5, 1}));
    EXPECT_TRUE(engine->IsValidLocation({0.5, 0.5, 100}));
}

TEST_F(FlatSquare, PointOutsideFootprintIsInvalid)
{
    EXPECT_FALSE(engine->IsValidLocation({20, 20, 1}));
    EXPECT_FALSE(engine->IsValidLocation({-1, 5, 1}));
}

TEST_F(FlatSquare, StraightPathCostIsEuclidean)
{
    // Both endpoints inside the lower-right triangle (y < x): single-face path.
    // --> euclidian distance in 2D (flat)
    const Point3D source{6, 2, 1};
    const Point3D target{9, 5, 1};

    const auto [path, cost] = engine->GetShortestPath(source, target);

    EXPECT_NEAR(cost, std::sqrt(3. * 3. + 3. * 3.), 1e-6);
    ASSERT_EQ(path.size(), 2u);
    // Endpoints keep the query x/y and are projected onto the surface (z=0).
    EXPECT_NEAR(path.front().x(), source.x(), 1e-6);
    EXPECT_NEAR(path.front().y(), source.y(), 1e-6);
    EXPECT_NEAR(path.front().z(), 0, 1e-6); // projected point has z=0
    EXPECT_NEAR(path.back().x(), target.x(), 1e-6);
    EXPECT_NEAR(path.back().y(), target.y(), 1e-6);
    EXPECT_NEAR(path.back().z(), 0, 1e-6); // projected point has z=0
}

TEST_F(FlatSquare, CrossingInternalEdgeStaysStraight)
{
    // Still straight line (no obstacles), crossing triangle boundaries.
    const Point3D source{2, 3, 1};
    const Point3D target{8, 7, 1};

    const auto [path, cost] = engine->GetShortestPath(source, target);

    EXPECT_NEAR(cost, std::sqrt(6. * 6. + 4. * 4.), 1e-6);
    // CGAL emits waypoints at each face edge it crosses, therefore not just 2 points returned.
    // But all points need to be collinear as it is a straight line.
    ASSERT_EQ(path.size(), 3u);
    EXPECT_TRUE(PointsCollinearXY(path));
}

TEST_F(FlatSquare, OrientationPointsToTarget)
{
    const Point dir = engine->GetOrientation({6, 2, 1}, {9, 5, 1});

    // Direction to the target (3, 3) normalized.
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(dir.x, inv_sqrt2, 1e-6);
    EXPECT_NEAR(dir.y, inv_sqrt2, 1e-6);
}

TEST_F(FlatSquare, OrientationRobustWhenSourceOnEdge)
{
    // source sits exactly on the shared diagonal (y=x). CGAL then emits a
    // duplicate leading waypoint; GetOrientation must skip it and still return
    // the real heading instead of a spurious (0,0).
    const Point3D source{4, 4, 1};

    const Point dir = engine->GetOrientation(source, {8, 7, 1});

    // Heading towards (8,7) from (4,4): (4,3) normalized = (0.8, 0.6).
    EXPECT_NEAR(dir.x, 0.8, 1e-6);
    EXPECT_NEAR(dir.y, 0.6, 1e-6);
}

TEST(RoutingEngine3DFold, GeodesicCarriesLengthAcrossSeam)
{
    Geometry3D geometry{};
    geometry.initialize_from_mesh(folded_mesh());
    SurfaceMeshShortestPathRoutingEngine engine{geometry};

    const Point3D source{3, 2, 1}; // on the floor
    const Point3D target{4, 13, 5}; // on the ramp, projects to z = 3

    const auto [path, cost] = engine.GetShortestPath(source, target);

    // Unfold the ramp about the seam (y=10): the ramp point (4,13,3) lies at
    // surface distance (13-10)*sqrt(2) from the seam, so it maps to
    // (4, 10 + 3*sqrt(2)). The geodesic is the straight line to it.
    const double unfolded_y = 10.0 + 3.0 * std::sqrt(2.0);
    const double dx = 4.0 - 3.0;
    const double dy = unfolded_y - 2.0;
    EXPECT_NEAR(cost, std::sqrt(dx * dx + dy * dy), 1e-6);

    ASSERT_GE(path.size(), 3u);
    EXPECT_NEAR(path.front().x(), source.x(), 1e-6);
    EXPECT_NEAR(path.front().y(), source.y(), 1e-6);
    EXPECT_NEAR(path.back().x(), target.x(), 1e-6);
    EXPECT_NEAR(path.back().y(), target.y(), 1e-6);
    EXPECT_NEAR(path.back().z(), 3, 1e-6); // projected onto the ramp (z = y-10)

    // The geodesic crosses the fold at a waypoint on the seam (y=10, z=0).
    const auto seam = std::find_if(
        path.begin(), path.end(), [](const Point3D& p) { return std::abs(p.y() - 10.0) < 1e-6; });
    ASSERT_NE(seam, path.end()) << "geodesic does not cross the seam";
    EXPECT_NEAR(seam->z(), 0.0, 1e-6);

    // Straight within each planar region: collinear on the floor up to the seam,
    // and collinear on the ramp from the seam onwards (the xy-direction bends
    // only at the fold). The seam point belongs to both slices.
    const auto seam_idx = static_cast<std::size_t>(std::distance(path.begin(), seam));
    EXPECT_TRUE(PointsCollinearXY(std::span(path).first(seam_idx + 1)));
    EXPECT_TRUE(PointsCollinearXY(std::span(path).subspan(seam_idx)));
}

TEST(RoutingEngine3DLShape, GeodesicBendsAroundReflexCorner)
{
    // L-shape (CCW) with a single reflex corner at (1, 1).
    const std::vector<K::Point_2> outer{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};

    Geometry3D geometry{};
    geometry.initialize_from_mesh(mesh_from_polygon(outer)); // flat z = 0
    SurfaceMeshShortestPathRoutingEngine engine{geometry};

    const Point3D source{2.5, 0.5, 1};
    const Point3D target{0.5, 2.5, 1};

    const auto [path, cost] = engine.GetShortestPath(source, target);

    // Straight line would cut the missing quadrant (x>1, y>1), so the any-angle
    // geodesic must pivot on the reflex corner (1,1):
    //   |S->(1,1)| + |(1,1)->T| = sqrt(2.5) + sqrt(2.5) = 2*sqrt(2.5).
    EXPECT_NEAR(cost, 2.0 * std::sqrt(2.5), 1e-6);
    ASSERT_EQ(path.size(), 3u);
    EXPECT_NEAR(path[1].x(), 1.0, 1e-6);
    EXPECT_NEAR(path[1].y(), 1.0, 1e-6);
}

TEST(RoutingEngine3DLShape, NextWaypointIsTheReflexCorner)
{
    const std::vector<K::Point_2> outer{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};

    Geometry3D geometry{};
    geometry.initialize_from_mesh(mesh_from_polygon(outer)); // flat z = 0
    SurfaceMeshShortestPathRoutingEngine engine{geometry};

    // The geodesic pivots exactly on the reflex corner -- no wall clearance
    // (unlike the legacy 2D funnel, which keeps 0.2 m off).
    const Point wp = engine.GetNextWaypoint({2.5, 0.5, 1}, {0.5, 2.5, 1});
    EXPECT_NEAR(wp.x, 1.0, 1e-6);
    EXPECT_NEAR(wp.y, 1.0, 1e-6);

    // Already at the target: the waypoint degenerates to the source itself.
    const Point at_goal = engine.GetNextWaypoint({2.5, 0.5, 1}, {2.5, 0.5, 1});
    EXPECT_NEAR(at_goal.x, 2.5, 1e-9);
    EXPECT_NEAR(at_goal.y, 0.5, 1e-9);
}

TEST(RoutingEngine2DAsRoutingEngine3D, DelegatesToTheLegacyEngineIgnoringZ)
{
    // The legacy 2D engine behind the 3D interface: identical answers to its
    // native 2D methods, z ignored on queries and 0 on results.
    const std::vector<K::Point_2> outer{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};
    RoutingEngine legacy{PolyWithHoles{Poly(outer.begin(), outer.end())}};
    RoutingEngine3D& engine = legacy;

    EXPECT_TRUE(engine.IsValidLocation({0.5, 0.5, 7.0})); // z ignored
    EXPECT_FALSE(engine.IsValidLocation({2.5, 2.5, 0.0})); // missing quadrant

    const Point source2d{2.5, 0.5};
    const Point target2d{0.5, 2.5};
    const Point3D source{source2d.x, source2d.y, 3.0};
    const Location target{target2d.x, target2d.y, -1.0};

    const auto expected_wp = legacy.ComputeWaypoint(source2d, target2d);
    const Point wp = engine.GetNextWaypoint(source, target);
    EXPECT_DOUBLE_EQ(wp.x, expected_wp.x);
    EXPECT_DOUBLE_EQ(wp.y, expected_wp.y);
    // The funnel's 0.2 m clearance shows: the waypoint is NOT the corner (1,1).
    EXPECT_GT((wp - Point{1, 1}).Norm(), 0.1);

    const auto expected_path = legacy.ComputeAllWaypoints(source2d, target2d);
    const auto [path, cost] = engine.GetShortestPath(source, target);
    ASSERT_EQ(path.size(), expected_path.size());
    double expected_cost = 0;
    for(std::size_t i = 0; i < path.size(); ++i) {
        EXPECT_DOUBLE_EQ(path[i].x(), expected_path[i].x);
        EXPECT_DOUBLE_EQ(path[i].y(), expected_path[i].y);
        EXPECT_EQ(path[i].z(), 0.0); // lifted flat
        if(i > 0) {
            expected_cost += (expected_path[i] - expected_path[i - 1]).Norm();
        }
    }
    EXPECT_DOUBLE_EQ(cost, expected_cost);

    const Point dir = engine.GetOrientation(source, target);
    const Point expected_dir = (expected_wp - source2d).Normalized();
    EXPECT_DOUBLE_EQ(dir.x, expected_dir.x);
    EXPECT_DOUBLE_EQ(dir.y, expected_dir.y);
}
