// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Geometry3D.hpp"

#include <gtest/gtest.h>

#include <array>
#include <set>

namespace
{
/// Two triangles sharing a diagonal; corners given counter-clockwise.
void add_quad(SurfaceMesh& mesh, const std::array<Point3D, 4>& corners)
{
    const auto v0 = mesh.add_vertex(corners[0]);
    const auto v1 = mesh.add_vertex(corners[1]);
    const auto v2 = mesh.add_vertex(corners[2]);
    const auto v3 = mesh.add_vertex(corners[3]);
    mesh.add_face(v0, v1, v2);
    mesh.add_face(v0, v2, v3);
}

SurfaceMesh flat_room()
{
    SurfaceMesh mesh{};
    add_quad(mesh, {Point3D{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}});
    return mesh;
}

/// A ramp climbing from z=0 at y=0 to z=4 at y=10 (so z = 0.4*y), one region.
SurfaceMesh ramp()
{
    SurfaceMesh mesh{};
    add_quad(mesh, {Point3D{5, 0, 0}, {15, 0, 0}, {15, 10, 4}, {5, 10, 4}});
    return mesh;
}

/// A flat square split into four triangles by a centre vertex, so a straight
/// walk can pass exactly *through* that vertex from the bottom triangle to the
/// non-adjacent top triangle -- the corner/fan case for walk_on_surface.
SurfaceMesh fan_square()
{
    SurfaceMesh mesh{};
    const auto c00 = mesh.add_vertex(Point3D{0, 0, 0});
    const auto c20 = mesh.add_vertex(Point3D{2, 0, 0});
    const auto c22 = mesh.add_vertex(Point3D{2, 2, 0});
    const auto c02 = mesh.add_vertex(Point3D{0, 2, 0});
    const auto ctr = mesh.add_vertex(Point3D{1, 1, 0});
    mesh.add_face(c00, c20, ctr); // bottom
    mesh.add_face(c20, c22, ctr); // right
    mesh.add_face(c22, c02, ctr); // top
    mesh.add_face(c02, c00, ctr); // left
    return mesh;
}

/// Two disjoint floors sharing the same (x,y) footprint at different heights:
/// the canonical stacking case that (x,y) alone cannot resolve.
SurfaceMesh stacked_floors()
{
    SurfaceMesh mesh{};
    add_quad(mesh, {Point3D{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}});
    add_quad(mesh, {Point3D{0, 0, 3}, {10, 0, 3}, {10, 10, 3}, {0, 10, 3}});
    return mesh;
}

} // namespace

TEST(Geometry3DLocate, FlatRegionYieldsGroundHeight)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(flat_room());
    ASSERT_EQ(geo.region_count(), 1);

    const auto loc = geo.locate_in_region(0, {5, 5});
    ASSERT_NE(loc.face, SurfaceMesh::null_face());
    EXPECT_NEAR(loc.point.z(), 0.0, 1e-9);
    EXPECT_NEAR(loc.point.x(), 5.0, 1e-9);
    EXPECT_NEAR(loc.point.y(), 5.0, 1e-9);
}

TEST(Geometry3DLocate, PointOutsideRegionFootprintMisses)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(flat_room());
    EXPECT_EQ(geo.locate_in_region(0, {20, 20}).face, SurfaceMesh::null_face());
}

TEST(Geometry3DLocate, RampInterpolatesHeightOnFace)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(ramp());
    ASSERT_EQ(geo.region_count(), 1);

    EXPECT_NEAR(geo.locate_in_region(0, {10, 2}).point.z(), 0.8, 1e-9);
    EXPECT_NEAR(geo.locate_in_region(0, {10, 9}).point.z(), 3.6, 1e-9);
}

TEST(Geometry3DLocate, RegionIdDisambiguatesStackedFloors)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(stacked_floors());
    ASSERT_EQ(geo.region_count(), 2);

    // Same (x,y), two sheets: the region id picks which one.
    EXPECT_NEAR(geo.locate_in_region(0, {5, 5}).point.z(), 0.0, 1e-9);
    EXPECT_NEAR(geo.locate_in_region(1, {5, 5}).point.z(), 3.0, 1e-9);
}

TEST(Geometry3DWalk, WalkAcrossInteriorDiagonalKeepsRegionAndInterpolatesHeight)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(ramp());
    ASSERT_EQ(geo.region_count(), 1);

    // (10,2) and (10,8) sit in the two triangles of the ramp quad; the straight
    // walk crosses the shared diagonal, stays in the one region, z tracks 0.4*y.
    const auto loc = geo.walk_on_surface(0, {10, 2}, {10, 8});
    ASSERT_NE(loc.face, SurfaceMesh::null_face());
    EXPECT_EQ(geo.region_of(loc.face), 0u);
    EXPECT_NEAR(loc.point.x(), 10.0, 1e-9);
    EXPECT_NEAR(loc.point.y(), 8.0, 1e-9);
    EXPECT_NEAR(loc.point.z(), 3.2, 1e-9);
}

TEST(Geometry3DWalk, WalkReachesVertexNeighbourFace)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(fan_square());
    ASSERT_EQ(geo.region_count(), 1);

    // Target (1,1.7) sits in the top triangle, which touches the start (bottom)
    // triangle only at the centre vertex (1,1) -- an edge search would miss it;
    // the vertex 1-ring covers it.
    const auto loc = geo.walk_on_surface(0, {1, 0.3}, {1, 1.7});
    ASSERT_NE(loc.face, SurfaceMesh::null_face());
    EXPECT_NEAR(loc.point.x(), 1.0, 1e-9);
    EXPECT_NEAR(loc.point.y(), 1.7, 1e-9);
    EXPECT_NEAR(loc.point.z(), 0.0, 1e-9);
}

TEST(Geometry3DWalk, WalkStartingExactlyOnVertexResolvesToNeighbour)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(fan_square());

    // 'from' is exactly the centre vertex: locate_in_region returns an arbitrary
    // incident face, but the vertex 1-ring still finds whichever fan face holds
    // 'to' (here the left triangle around (0.3,1)).
    const auto loc = geo.walk_on_surface(0, {1, 1}, {0.3, 1});
    ASSERT_NE(loc.face, SurfaceMesh::null_face());
    EXPECT_NEAR(loc.point.x(), 0.3, 1e-9);
    EXPECT_NEAR(loc.point.y(), 1.0, 1e-9);
    EXPECT_NEAR(loc.point.z(), 0.0, 1e-9);
}

TEST(Geometry3DWalk, WalkWithinOneTriangleReturnsTargetHeight)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(ramp());

    // Both endpoints in the same triangle -> resolved by the initial locate.
    const auto loc = geo.walk_on_surface(0, {10, 2}, {11, 3});
    ASSERT_NE(loc.face, SurfaceMesh::null_face());
    EXPECT_NEAR(loc.point.x(), 11.0, 1e-9);
    EXPECT_NEAR(loc.point.y(), 3.0, 1e-9);
    EXPECT_NEAR(loc.point.z(), 1.2, 1e-9); // 0.4*3
}

TEST(Geometry3DWalk, WalkTargetOutsideNeighbourhoodThrows)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(flat_room());

    // 'to' far outside the start face and its 1-ring: a step too large for the
    // mesh resolution (or off-surface) -- rejected rather than silently wrong.
    EXPECT_ANY_THROW(geo.walk_on_surface(0, {3, 2}, {20, 20}));
}
