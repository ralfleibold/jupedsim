// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "GeometryBuilder.hpp"
#include "Journey.hpp"
#include "OperationalModels/CollisionFreeSpeedModel/CollisionFreeSpeedModel.hpp"
#include "RoutingEngine.hpp"
#include "Simulation.hpp"
#include "SimulationError.hpp"
#include "StageDescription.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <vector>

namespace
{
/// L-corridor with the inner reflex corner at (8, 2); one agent at (1, 1)
/// heads for an exit at the top of the vertical leg -- its route has to bend
/// around that corner. @p surfaceRouting picks which engine is injected: the
/// surface-mesh geodesic on the flat lift, or the legacy 2D engine.
/// @p runIn2d selects the operational path (legacy 2D vs. surface step).
std::unique_ptr<Simulation> l_corridor_simulation(bool surfaceRouting, bool runIn2d = true)
{
    GeometryBuilder builder{};
    builder.AddAccessibleArea({{0, 0}, {10, 0}, {10, 10}, {8, 10}, {8, 2}, {0, 2}});
    auto geometry = std::make_unique<Geometry3D>();
    geometry->initialize_from_polygon(builder.Build().Polygon());

    std::unique_ptr<RoutingEngine3D> routingEngine{};
    if(surfaceRouting) {
        routingEngine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(*geometry);
    } else {
        routingEngine = std::make_unique<RoutingEngine>(geometry->collision_geometry()->Polygon());
    }

    auto simulation = std::make_unique<Simulation>(
        std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
        std::move(geometry),
        std::move(routingEngine),
        0.01,
        runIn2d);

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

TEST(Simulation, UsesTheInjectedRoutingEngine)
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

TEST(Simulation, SurfaceOperationalPathReproducesThe2DPath)
{
    // The parity gate for the switch: identical routing (legacy TA*), only
    // the operational path differs -- gather/apply on the 2D structures vs.
    // gather/apply/re-anchor on the flat surface lift. Trajectories must
    // match exactly, wall contact at the corner included.
    const auto in2d = l_corridor_simulation(false, true);
    const auto onSurface = l_corridor_simulation(false, false);

    for(int step = 0; step < 800; ++step) {
        in2d->Iterate();
        onSurface->Iterate();
        ASSERT_EQ(in2d->AgentCount(), 1u);
        ASSERT_EQ(onSurface->AgentCount(), 1u);
        const auto& expected = in2d->Agents().front();
        const auto& actual = onSurface->Agents().front();
        ASSERT_NEAR(actual.pos.x, expected.pos.x, 1e-9) << "step " << step;
        ASSERT_NEAR(actual.pos.y, expected.pos.y, 1e-9) << "step " << step;
        ASSERT_EQ(actual.regionId, 0u);
    }
    // Sanity: the walk is well underway, past the corner region.
    EXPECT_GT(in2d->Agents().front().pos.x, 5.0);
}

TEST(Simulation, ValidatesNewAgentsInBothModes)
{
    // Adding happens outside Iterate, so surface mode must see already-added
    // agents through the incrementally maintained index, not only through the
    // per-step update. Verdicts must match the 2D path.
    for(const bool runIn2d : {true, false}) {
        SCOPED_TRACE(runIn2d ? "2d" : "surface");
        const auto simulation = l_corridor_simulation(false, runIn2d);
        const auto& first = simulation->Agents().front();
        const auto agent_at = [journeyId = first.journeyId,
                               stageId = first.stageId](Point pos) {
            return GenericAgent(
                GenericAgent::ID::Invalid, journeyId, stageId, pos, CollisionFreeSpeedModelData{});
        };

        // Outside the walkable area.
        EXPECT_THROW(simulation->AddAgent(agent_at({-50, -50})), SimulationError);
        // On top of the initial agent (contact distance 0.4).
        EXPECT_THROW(simulation->AddAgent(agent_at({1, 1})), SimulationError);
        // Closer to the wall y=0 than the agent radius 0.2.
        EXPECT_THROW(simulation->AddAgent(agent_at({3, 0.1})), SimulationError);

        // Clear of the agent and all walls.
        EXPECT_NO_THROW(simulation->AddAgent(agent_at({3, 1})));
        // The agent just added is itself visible to validation right away.
        EXPECT_THROW(simulation->AddAgent(agent_at({3.1, 1})), SimulationError);
    }
}

TEST(Simulation, RejectsGeometryWithoutThe2DView)
{
    // Built from a mesh, not a polygon: no projected 2D view. The operational
    // layer still needs one, so the constructor must refuse.
    SurfaceMesh mesh{};
    const auto v0 = mesh.add_vertex({0, 0, 0});
    const auto v1 = mesh.add_vertex({10, 0, 0});
    const auto v2 = mesh.add_vertex({10, 10, 0});
    mesh.add_face(v0, v1, v2);
    auto geometry = std::make_unique<Geometry3D>();
    geometry->initialize_from_mesh(std::move(mesh));
    auto routingEngine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(*geometry);

    EXPECT_THROW(
        Simulation(
            std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
            std::move(geometry),
            std::move(routingEngine),
            0.01),
        SimulationError);
}
