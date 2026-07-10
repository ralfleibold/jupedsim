// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Simulation.hpp"

#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "GeometryBuilder.hpp"
#include "Journey.hpp"
#include "OperationalModels/CollisionFreeSpeedModel/CollisionFreeSpeedModel.hpp"
#include "StageDescription.hpp"

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <vector>

namespace
{
/// L-corridor with the inner reflex corner at (8, 2); one agent at (1, 1)
/// heads for an exit at the top of the vertical leg -- its route has to bend
/// around that corner. @p surfaceRouting selects the simulation's routing
/// engine via the transitional Geometry3D seam.
std::unique_ptr<Simulation> l_corridor_simulation(bool surfaceRouting)
{
    GeometryBuilder builder{};
    builder.AddAccessibleArea({{0, 0}, {10, 0}, {10, 10}, {8, 10}, {8, 2}, {0, 2}});
    auto geometry = std::make_unique<CollisionGeometry>(builder.Build());

    std::unique_ptr<Geometry3D> geometry3d{};
    if(surfaceRouting) {
        geometry3d = std::make_unique<Geometry3D>();
        geometry3d->initialize_from_polygon(geometry->Polygon());
    }

    auto simulation = std::make_unique<Simulation>(
        std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
        std::move(geometry),
        0.01,
        std::move(geometry3d));

    const auto exitId = simulation->AddStage(
        ExitDescription{Polygon{std::vector<Point>{{8, 9}, {10, 9}, {10, 10}, {8, 10}}}});
    const auto journeyId = simulation->AddJourney({{exitId, NonTransitionDescription{}}});
    simulation->AddAgent(GenericAgent(
        GenericAgent::ID::Invalid, journeyId, exitId, {1, 1}, CollisionFreeSpeedModelData{}));
    return simulation;
}

/// The agent's routed waypoint after one full iteration (strategical sets the
/// target, tactical the destination).
Point destination_after_one_iteration(Simulation& simulation)
{
    simulation.Iterate();
    return simulation.Agents().front().destination;
}
} // namespace

TEST(Simulation, SurfaceRoutingSeamSelectsTheRoutingEngine)
{
    const Point corner{8, 2};

    // Legacy TA* engine: the funnel keeps 0.2 m clearance at portal ends, so
    // the waypoint is near, but never on, the corner.
    const auto legacy = l_corridor_simulation(false);
    const auto legacy_destination = destination_after_one_iteration(*legacy);
    EXPECT_GT((legacy_destination - corner).Norm(), 0.1);
    EXPECT_LT((legacy_destination - corner).Norm(), 1.0);

    // Surface-mesh geodesic on the flat lift: the path pivots exactly on the
    // corner vertex.
    const auto surface = l_corridor_simulation(true);
    const auto surface_destination = destination_after_one_iteration(*surface);
    EXPECT_NEAR(surface_destination.x, corner.x, 1e-6);
    EXPECT_NEAR(surface_destination.y, corner.y, 1e-6);
}
