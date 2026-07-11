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

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
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
        const auto agent_at = [journeyId = first.journeyId, stageId = first.stageId](Point pos) {
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

TEST(Simulation, ValidatesNewStagesInBothModes)
{
    // AddStage rejects positions off the walkable area -- via InsideGeometry
    // in 2D mode, via the z-hint surface locate in surface mode. Verdicts
    // must match for every stage flavor.
    for(const bool runIn2d : {true, false}) {
        SCOPED_TRACE(runIn2d ? "2d" : "surface");
        const auto simulation = l_corridor_simulation(false, runIn2d);

        EXPECT_NO_THROW(simulation->AddStage(WaypointDescription{{5, 1}, 0.5}));
        EXPECT_THROW(simulation->AddStage(WaypointDescription{{5, 5}, 0.5}), SimulationError);

        EXPECT_NO_THROW(simulation->AddStage(
            ExitDescription{Polygon{std::vector<Point>{{0, 0}, {2, 0}, {2, 2}, {0, 2}}}}));
        // Centroid (5, 5) is in the corridor's notch.
        EXPECT_THROW(
            simulation->AddStage(
                ExitDescription{Polygon{std::vector<Point>{{4, 4}, {6, 4}, {6, 6}, {4, 6}}}}),
            SimulationError);

        EXPECT_NO_THROW(simulation->AddStage(NotifiableWaitingSetDescription{{{5, 1}, {6, 1}}}));
        EXPECT_THROW(
            simulation->AddStage(NotifiableWaitingSetDescription{{{5, 1}, {5, 5}}}),
            SimulationError);

        EXPECT_NO_THROW(simulation->AddStage(NotifiableQueueDescription{{{5, 1}, {6, 1}}}));
        EXPECT_THROW(
            simulation->AddStage(NotifiableQueueDescription{{{5, 1}, {5, 5}}}), SimulationError);
    }
}

TEST(Simulation, PublicAgentQueriesMatchAcrossModes)
{
    // AgentsInRange / AgentsInPolygon answer from the mode's own bookkeeping
    // (2D grid vs. surface scan); membership must be identical.
    for(const bool runIn2d : {true, false}) {
        SCOPED_TRACE(runIn2d ? "2d" : "surface");
        const auto simulation = l_corridor_simulation(false, runIn2d);
        const auto& first = simulation->Agents().front();
        for(const auto x : {3.0, 6.0}) {
            simulation->AddAgent(GenericAgent(
                GenericAgent::ID::Invalid,
                first.journeyId,
                first.stageId,
                {x, 1},
                CollisionFreeSpeedModelData{}));
        }

        const auto xs_of = [&simulation](const std::vector<GenericAgent::ID>& ids) {
            std::vector<double> xs{};
            for(const auto id : ids) {
                xs.push_back(simulation->Agent(id).pos.x);
            }
            std::sort(xs.begin(), xs.end());
            return xs;
        };
        // Around (1,1) with radius 2.5: the agents at x=1 and x=3.
        EXPECT_EQ(xs_of(simulation->AgentsInRange({1, 1}, 2.5)), (std::vector<double>{1, 3}));
        // The box [2,7]x[0,2]: the agents at x=3 and x=6.
        EXPECT_EQ(
            xs_of(simulation->AgentsInPolygon({{2, 0}, {7, 0}, {7, 2}, {2, 2}})),
            (std::vector<double>{3, 6}));
    }
}

TEST(Simulation, QueueBehaviorMatchesAcrossModes)
{
    // A queue exercises the stage system's world queries (slot surroundings,
    // wall visibility) each step -- through the 2D grid in one mode and the
    // surface gatherer in the other. Occupancy decisions feed back into the
    // walk, so identical trajectories mean identical stage behavior. The pop
    // and the following exit also cover agent removal (which forces the
    // surface index rebuild at the start of the next iteration).
    const auto build = [](bool runIn2d) {
        GeometryBuilder builder{};
        builder.AddAccessibleArea({{0, 0}, {10, 0}, {10, 10}, {8, 10}, {8, 2}, {0, 2}});
        auto geometry = std::make_unique<Geometry3D>();
        geometry->initialize_from_polygon(builder.Build().Polygon());
        auto routingEngine =
            std::make_unique<RoutingEngine>(geometry->collision_geometry()->Polygon());
        auto simulation = std::make_unique<Simulation>(
            std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
            std::move(geometry),
            std::move(routingEngine),
            0.01,
            runIn2d);
        const auto queueId =
            simulation->AddStage(NotifiableQueueDescription{std::vector<Point>{{6, 1}, {5, 1}}});
        const auto exitId = simulation->AddStage(
            ExitDescription{Polygon{std::vector<Point>{{8, 9}, {10, 9}, {10, 10}, {8, 10}}}});
        const auto journeyId = simulation->AddJourney(
            {{queueId, FixedTransitionDescription{exitId}}, {exitId, NonTransitionDescription{}}});
        for(const auto x : {1.0, 2.0}) {
            simulation->AddAgent(GenericAgent(
                GenericAgent::ID::Invalid,
                journeyId,
                queueId,
                {x, 1},
                CollisionFreeSpeedModelData{}));
        }
        return std::make_pair(std::move(simulation), queueId);
    };

    auto [in2d, queue2dId] = build(true);
    auto [onSurface, queueSurfaceId] = build(false);

    const auto compare_step = [&](int step) {
        ASSERT_EQ(in2d->AgentCount(), onSurface->AgentCount()) << "step " << step;
        auto expected = in2d->Agents().begin();
        for(const auto& actual : onSurface->Agents()) {
            ASSERT_NEAR(actual.pos.x, expected->pos.x, 1e-9) << "step " << step;
            ASSERT_NEAR(actual.pos.y, expected->pos.y, 1e-9) << "step " << step;
            ++expected;
        }
    };

    for(int step = 0; step < 600; ++step) {
        in2d->Iterate();
        onSurface->Iterate();
        compare_step(step);
    }
    // Both queues filled up the same way.
    auto queue2d = std::get<NotifiableQueueProxy>(in2d->Stage(queue2dId));
    auto queueSurface = std::get<NotifiableQueueProxy>(onSurface->Stage(queueSurfaceId));
    ASSERT_EQ(queue2d.CountEnqueued(), 2u);
    ASSERT_EQ(queueSurface.CountEnqueued(), 2u);

    // Release the head of both queues; it walks off to the exit and is
    // removed, the second agent advances.
    queue2d.Pop(1);
    queueSurface.Pop(1);
    for(int step = 0; step < 1500; ++step) {
        in2d->Iterate();
        onSurface->Iterate();
        compare_step(step);
    }
    EXPECT_EQ(in2d->AgentCount(), 1u);
    EXPECT_EQ(onSurface->AgentCount(), 1u);
}

TEST(Simulation, The2DViewIsRequiredOnlyOnThe2DPath)
{
    // Built from a mesh, not a polygon: no projected 2D view. The 2D
    // operational path needs one for collision handling; the surface path
    // runs entirely on the mesh and must accept such geometry.
    const auto build = [](bool runIn2d) {
        SurfaceMesh mesh{};
        const auto v0 = mesh.add_vertex({0, 0, 0});
        const auto v1 = mesh.add_vertex({10, 0, 0});
        const auto v2 = mesh.add_vertex({10, 10, 0});
        mesh.add_face(v0, v1, v2);
        auto geometry = std::make_unique<Geometry3D>();
        geometry->initialize_from_mesh(std::move(mesh));
        auto routingEngine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(*geometry);
        return std::make_unique<Simulation>(
            std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
            std::move(geometry),
            std::move(routingEngine),
            0.01,
            runIn2d);
    };

    EXPECT_THROW(build(true), SimulationError);
    EXPECT_NO_THROW(build(false));
}

namespace
{
/// Two-storey geometry as ONE connected mesh (the TestSurfaceStep shape):
/// ground floor (x in [0,16], y in [0,6], z=0) -> ramp (x in [12,16],
/// climbing y 6..12 to z=3) -> upper floor in a U whose last part (x in
/// [6,12], y in [0,12], z=3) lies directly above the ground floor. The
/// (x,y) overlap forces a two-region split.
SurfaceMesh two_storey_mesh()
{
    SurfaceMesh mesh{};
    const auto a = mesh.add_vertex(Point3D{0, 0, 0});
    const auto b = mesh.add_vertex(Point3D{12, 0, 0});
    const auto c = mesh.add_vertex(Point3D{16, 0, 0});
    const auto d = mesh.add_vertex(Point3D{16, 6, 0});
    const auto e = mesh.add_vertex(Point3D{12, 6, 0});
    const auto f = mesh.add_vertex(Point3D{0, 6, 0});
    const auto r1 = mesh.add_vertex(Point3D{16, 12, 3});
    const auto r2 = mesh.add_vertex(Point3D{12, 12, 3});
    const auto u1 = mesh.add_vertex(Point3D{16, 16, 3});
    const auto u2 = mesh.add_vertex(Point3D{12, 16, 3});
    const auto u3 = mesh.add_vertex(Point3D{6, 16, 3});
    const auto u4 = mesh.add_vertex(Point3D{6, 12, 3});
    const auto u5 = mesh.add_vertex(Point3D{6, 0, 3});
    const auto u6 = mesh.add_vertex(Point3D{12, 0, 3});

    const auto add_quad_faces = [&mesh](auto v0, auto v1, auto v2, auto v3) {
        mesh.add_face(v0, v1, v2);
        mesh.add_face(v0, v2, v3);
    };
    add_quad_faces(a, b, e, f); // ground left
    add_quad_faces(b, c, d, e); // ground right
    add_quad_faces(e, d, r1, r2); // ramp
    add_quad_faces(r2, r1, u1, u2); // upper, ahead of the ramp
    add_quad_faces(u4, r2, u2, u3); // upper, turning left
    add_quad_faces(u5, u6, r2, u4); // upper, back over the ground
    return mesh;
}
} // namespace

TEST(Simulation, AddAgentSelectsTheSheetByZHint)
{
    auto geometry = std::make_unique<Geometry3D>();
    geometry->initialize_from_mesh(two_storey_mesh());
    ASSERT_EQ(geometry->region_count(), 2u);
    const auto* geo = geometry.get();
    auto routingEngine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(*geometry);
    Simulation simulation(
        std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
        std::move(geometry),
        std::move(routingEngine),
        0.01,
        false);

    // Exit on the ground floor, outside the overlapped strip x in [6,12].
    const auto exitId = simulation.AddStage(
        ExitDescription{Polygon{std::vector<Point>{{0, 0}, {2, 0}, {2, 2}, {0, 2}}}});
    const auto journeyId = simulation.AddJourney({{exitId, NonTransitionDescription{}}});
    const auto agent_at = [journeyId, exitId](Point pos) {
        return GenericAgent(
            GenericAgent::ID::Invalid, journeyId, exitId, pos, CollisionFreeSpeedModelData{});
    };

    // (9, 3) has two sheets: the ground floor at z=0 and the upper floor at
    // z=3. The z-hint picks the sheet; the two agents are stacked in (x,y)
    // yet do not collide (outside the vertical interaction band).
    const auto groundId = simulation.AddAgent(agent_at({9, 3}));
    const auto upperId = simulation.AddAgent(agent_at({9, 3}), 3.1);

    const auto sheet_z = [geo, &simulation](GenericAgent::ID id) {
        const auto& agent = simulation.Agent(id);
        return geo->locate_in_region(agent.regionId, {agent.pos.x, agent.pos.y}).point.z();
    };
    EXPECT_DOUBLE_EQ(sheet_z(groundId), 0.0);
    EXPECT_DOUBLE_EQ(sheet_z(upperId), 3.0);

    // No sheet within the +/- 0.25 m tolerance of the hint.
    EXPECT_THROW(simulation.AddAgent(agent_at({9, 3}), 1.5), SimulationError);
    // Off the walkable area entirely.
    EXPECT_THROW(simulation.AddAgent(agent_at({-50, -50})), SimulationError);
}

TEST(Simulation, GroundExitDoesNotSwallowAgentOnTheFloorAbove)
{
    auto geometry = std::make_unique<Geometry3D>();
    geometry->initialize_from_mesh(two_storey_mesh());
    auto routingEngine = std::make_unique<SurfaceMeshShortestPathRoutingEngine>(*geometry);
    Simulation simulation(
        std::make_unique<CollisionFreeSpeedModel>(CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02}),
        std::move(geometry),
        std::move(routingEngine),
        0.01,
        false);

    // Exit on the ground floor, directly under the upper floor.
    const auto exitId = simulation.AddStage(
        ExitDescription{Polygon{std::vector<Point>{{7, 0}, {9, 0}, {9, 2}, {7, 2}}}});
    const auto journeyId = simulation.AddJourney({{exitId, NonTransitionDescription{}}});
    const auto agent_at = [journeyId, exitId](Point pos) {
        return GenericAgent(
            GenericAgent::ID::Invalid, journeyId, exitId, pos, CollisionFreeSpeedModelData{});
    };

    // The first agent starts on the upper floor exactly above the exit: its
    // (x,y) is inside the exit polygon from the first step, 3 m too high.
    // Without the z-band around the exit's anchored height it would be
    // removed on the spot; instead it must keep walking toward the ramp.
    simulation.AddAgent(agent_at({8, 1}), 3.0);
    for(int step = 0; step < 30; ++step) {
        simulation.Iterate();
        ASSERT_EQ(simulation.AgentCount(), 1u) << "step " << step;
    }
    EXPECT_DOUBLE_EQ(simulation.Agents().front().z, 3.0);

    // A ground-floor agent placed inside the exit completes right away --
    // the z-band keeps same-floor completion intact. (It is stacked under
    // the live upper agent: no validation clash across sheets either.)
    simulation.AddAgent(agent_at({8, 1}));
    ASSERT_EQ(simulation.AgentCount(), 2u);
    simulation.Iterate();
    EXPECT_EQ(simulation.AgentCount(), 1u);
    EXPECT_DOUBLE_EQ(simulation.Agents().front().z, 3.0);
}
