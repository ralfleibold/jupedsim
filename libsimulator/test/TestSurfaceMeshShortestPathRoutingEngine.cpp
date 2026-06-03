// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CfgCgal.hpp"
#include "CollisionGeometry.hpp"
#include "Point.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
double path_length(const std::vector<Point>& path)
{
    double sum{};
    for(size_t i = 1; i < path.size(); ++i) {
        const auto dx = path[i].x - path[i - 1].x;
        const auto dy = path[i].y - path[i - 1].y;
        sum += std::sqrt(dx * dx + dy * dy);
    }
    return sum;
}

Poly makeSquare(double x0, double y0, double x1, double y1)
{
    const std::vector<K::Point_2> pts{
        K::Point_2{x0, y0},
        K::Point_2{x1, y0},
        K::Point_2{x1, y1},
        K::Point_2{x0, y1},
    };
    return Poly{pts.begin(), pts.end()};
}
} // namespace

TEST(SurfaceMeshShortestPathRoutingEngine, StraightLineInEmptyRoom)
{
    CollisionGeometry geometry{PolyWithHoles{makeSquare(0, 0, 10, 10)}};
    SurfaceMeshShortestPathRoutingEngine engine{};
    engine.set_geometry(geometry);

    const Point from{1, 1};
    const Point to{9, 9};
    const auto path = engine.compute_waypoints(from, to);

    ASSERT_GE(path.size(), 2u);
    EXPECT_NEAR(path.front().x, from.x, 1e-6);
    EXPECT_NEAR(path.front().y, from.y, 1e-6);
    EXPECT_NEAR(path.back().x, to.x, 1e-6);
    EXPECT_NEAR(path.back().y, to.y, 1e-6);
    // Unobstructed: geodesic is the straight segment.
    EXPECT_NEAR(path_length(path), std::sqrt(128.0), 1e-6);
}

TEST(SurfaceMeshShortestPathRoutingEngine, RoutesAroundCentralObstacle)
{
    PolyWithHoles pwh{makeSquare(0, 0, 10, 10)};
    pwh.add_hole(makeSquare(4, 4, 6, 6));
    CollisionGeometry geometry{pwh};
    SurfaceMeshShortestPathRoutingEngine engine{};
    engine.set_geometry(geometry);

    const Point from{1, 5};
    const Point to{9, 5};
    const auto path = engine.compute_waypoints(from, to);

    EXPECT_NEAR(path.front().x, from.x, 1e-6);
    EXPECT_NEAR(path.front().y, from.y, 1e-6);
    EXPECT_NEAR(path.back().x, to.x, 1e-6);
    EXPECT_NEAR(path.back().y, to.y, 1e-6);
    // Direct horizontal line (length 8) crosses the obstacle, so the geodesic must bend.
    ASSERT_GE(path.size(), 3u);
    EXPECT_GT(path_length(path), 8.0);
    // Tight reference path hugs the lower obstacle edge: (1,5)->(4,4)->(6,4)->(9,5),
    // length 2*sqrt(10) + 2.
    EXPECT_NEAR(path_length(path), 2.0 * std::sqrt(10.0) + 2.0, 1e-4);
}

TEST(SurfaceMeshShortestPathRoutingEngine, StripsCollinearWaypoints)
{
    PolyWithHoles pwh{makeSquare(0, 0, 10, 10)};
    pwh.add_hole(makeSquare(4, 4, 6, 6));
    CollisionGeometry geometry{pwh};
    SurfaceMeshShortestPathRoutingEngine engine{};
    engine.set_geometry(geometry);

    const auto path = engine.compute_waypoints(Point{1, 5}, Point{9, 5});

    // Corner-only path: start, two obstacle corners, destination. The geodesic crosses CDT
    // diagonals on the straight runs but those collinear points must be removed.
    ASSERT_EQ(path.size(), 4u);
    // No interior point may be collinear with its neighbours (within tolerance).
    for(size_t i = 1; i + 1 < path.size(); ++i) {
        const auto& a = path[i - 1];
        const auto& b = path[i];
        const auto& c = path[i + 1];
        const double cross =
            (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        const double ac_len = std::hypot(c.x - a.x, c.y - a.y);
        EXPECT_GT(std::abs(cross) / ac_len, 1e-7) << "collinear waypoint at index " << i;
    }
    // Length is unchanged by stripping.
    EXPECT_NEAR(path_length(path), 2.0 * std::sqrt(10.0) + 2.0, 1e-4);
}

TEST(SurfaceMeshShortestPathRoutingEngine, IsRoutable)
{
    PolyWithHoles pwh{makeSquare(0, 0, 10, 10)};
    pwh.add_hole(makeSquare(4, 4, 6, 6));
    CollisionGeometry geometry{pwh};
    SurfaceMeshShortestPathRoutingEngine engine{};
    engine.set_geometry(geometry);

    EXPECT_TRUE(engine.is_routable(Point{1, 1}));
    EXPECT_FALSE(engine.is_routable(Point{5, 5})); // inside the hole
    EXPECT_FALSE(engine.is_routable(Point{20, 20})); // outside the room
}
