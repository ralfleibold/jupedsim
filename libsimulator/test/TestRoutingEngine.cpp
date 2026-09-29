// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GeometryFixtures.hpp"
#include "Polygon.hpp"
#include "RoutingEngine.hpp"
#include "SimulationError.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(RoutingEngine, LeadsToTheNearestAreaOfADestination)
{
    const auto geometry = test_geometries::rectangle({0, 0}, {10, 2});
    RoutingEngine engine{*geometry};

    const std::vector<DestinationArea> exits{
        {0, Polygon{test_geometries::rectangle_points({0.5, 0.5}, {1.5, 1.5})}},
        {0, Polygon{test_geometries::rectangle_points({8.5, 0.5}, {9.5, 1.5})}}};
    const auto destination = engine.AddDestination(exits);

    const auto right = geometry->get_location(7, 1, 0).value();
    const auto left = geometry->get_location(3, 1, 0).value();
    EXPECT_GT(engine.GetOrientation(right, destination).x, 0.0);
    EXPECT_LT(engine.GetOrientation(left, destination).x, 0.0);
}

TEST(RoutingEngine, ADestinationNeedsAnAreaInsideItsRegion)
{
    const auto geometry = test_geometries::rectangle({0, 0}, {10, 2});
    RoutingEngine engine{*geometry};

    EXPECT_THROW(engine.AddDestination({}), SimulationError);
    const DestinationArea outside{0, Polygon{test_geometries::rectangle_points({20, 0}, {21, 1})}};
    EXPECT_THROW(engine.AddDestination({&outside, 1}), SimulationError);
}
