// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SurfaceStep.hpp"

#include "GenericAgent.hpp"
#include "GeometryBuilder.hpp"
#include "Geometry3D.hpp"
#include "InformationGatherer3D.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalDecisionSystem.hpp"
#include "OperationalModels/CollisionFreeSpeedModel/CollisionFreeSpeedModel.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace
{
constexpr double dT = 0.05;

/// CFSM with the library's default parameters.
CollisionFreeSpeedModel default_cfsm()
{
    return CollisionFreeSpeedModel{8.0, 0.1, 5.0, 0.02};
}

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

/// The flat_room with a bridge deck spanning x in [4,6] at height @p z.
SurfaceMesh flat_room_with_bridge(double z)
{
    SurfaceMesh mesh{};
    add_quad(mesh, {Point3D{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}});
    add_quad(mesh, {Point3D{4, 0, z}, {6, 0, z}, {6, 10, z}, {4, 10, z}});
    return mesh;
}

GenericAgent make_agent(const Point& pos, const Point& destination)
{
    auto agent = GenericAgent(
        GenericAgent::ID::Invalid,
        jps::UniqueID<Journey>::Invalid,
        jps::UniqueID<BaseStage>::Invalid,
        pos,
        CollisionFreeSpeedModelData{});
    agent.destination = destination;
    return agent;
}

/// On-surface anchor of the sheet at height @p z covering the agent position.
Geometry3D::FaceLocation
anchor_at_height(const Geometry3D& geo, const GenericAgent& agent, double z)
{
    for(std::size_t region = 0; region < geo.region_count(); ++region) {
        const auto loc = geo.locate_in_region(region, {agent.pos.x, agent.pos.y});
        if(loc.face != SurfaceMesh::null_face() && std::abs(loc.point.z() - z) < 1e-9) {
            return loc;
        }
    }
    ADD_FAILURE() << "no sheet at (" << agent.pos.x << ", " << agent.pos.y << ", z=" << z << ")";
    return {SurfaceMesh::null_face(), Point3D{}};
}
} // namespace

TEST(SurfaceStep, RampWalkRaisesHeightAndKeepsRegion)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(ramp());
    ASSERT_EQ(geo.region_count(), 1);

    const auto model = default_cfsm();
    AgentContainer<GenericAgent> agents{};
    agents.push_back(make_agent({10, 2}, {10, 8}));
    std::vector<Geometry3D::FaceLocation> anchors{
        anchor_at_height(geo, agents.front(), 0.4 * 2)};

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < 60; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents, anchors);
    }

    // v0=1.2 for 3 s moves the agent well up the ramp on a straight line.
    const auto& agent = agents.front();
    EXPECT_GT(agent.pos.y, 4.0);
    EXPECT_NEAR(agent.pos.x, 10.0, 1e-6);
    // The anchor tracks the agent, on the ramp plane z = 0.4*y, region 0.
    EXPECT_NEAR(anchors.front().point.x(), agent.pos.x, 1e-9);
    EXPECT_NEAR(anchors.front().point.y(), agent.pos.y, 1e-9);
    EXPECT_NEAR(anchors.front().point.z(), 0.4 * agent.pos.y, 1e-9);
    EXPECT_EQ(geo.region_of(anchors.front().face), 0u);
}

TEST(SurfaceStep, AgentWalksStraightBelowAnUpperFloor)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(flat_room_with_bridge(3.0));
    ASSERT_EQ(geo.region_count(), 2);

    // The ground agent passes under the bridge; the bridge agent stands right
    // above its path at the same (x,y). Were the vertical band ignored, the
    // bridge agent (spacing -> 0) would stall the ground agent and its borders
    // would deflect it sideways.
    const auto model = default_cfsm();
    AgentContainer<GenericAgent> agents{};
    agents.push_back(make_agent({3, 5}, {8, 5}));
    agents.push_back(make_agent({5, 5}, {5, 5}));
    std::vector<Geometry3D::FaceLocation> anchors{
        anchor_at_height(geo, agents[0], 0), anchor_at_height(geo, agents[1], 3.0)};

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < 60; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents, anchors);
    }

    EXPECT_GT(agents[0].pos.x, 6.5);
    EXPECT_NEAR(agents[0].pos.y, 5.0, 1e-6);
    EXPECT_NEAR(anchors[0].point.z(), 0.0, 1e-9);
    EXPECT_NEAR(anchors[1].point.z(), 3.0, 1e-9);
}

TEST(SurfaceStep, FlatGeometryReproduces2DPipeline)
{
    // The switch criterion: on a flat geometry the 3D pipeline must yield the
    // same trajectories as the 2D pipeline. Two interacting agents walk toward
    // a shared goal in a 10x10 room, once through OperationalDecisionSystem
    // (CollisionGeometry + NeighborhoodSearch), once through run_surface_step.
    const auto start_a = Point{2, 2};
    const auto start_b = Point{2.8, 2};
    const auto goal = Point{8, 8};
    constexpr int steps = 60;

    GeometryBuilder builder{};
    builder.AddAccessibleArea({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    const auto geometry2d = builder.Build();

    AgentContainer<GenericAgent> agents2d{};
    agents2d.push_back(make_agent(start_a, goal));
    agents2d.push_back(make_agent(start_b, goal));

    OperationalDecisionSystem system{std::make_unique<CollisionFreeSpeedModel>(default_cfsm())};
    NeighborhoodSearch<GenericAgent> neighborhoodSearch{2.2};
    for(int step = 0; step < steps; ++step) {
        neighborhoodSearch.Update(agents2d);
        system.Run(dT, 0., neighborhoodSearch, geometry2d, agents2d);
    }

    Geometry3D geo{};
    geo.initialize_from_mesh(flat_room());
    const auto model = default_cfsm();
    AgentContainer<GenericAgent> agents3d{};
    agents3d.push_back(make_agent(start_a, goal));
    agents3d.push_back(make_agent(start_b, goal));
    std::vector<Geometry3D::FaceLocation> anchors{
        anchor_at_height(geo, agents3d[0], 0), anchor_at_height(geo, agents3d[1], 0)};

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < steps; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents3d, anchors);
    }

    for(std::size_t i = 0; i < agents2d.size(); ++i) {
        EXPECT_NEAR(agents3d[i].pos.x, agents2d[i].pos.x, 1e-9) << "agent " << i;
        EXPECT_NEAR(agents3d[i].pos.y, agents2d[i].pos.y, 1e-9) << "agent " << i;
    }
    // Sanity: they actually moved.
    EXPECT_GT(agents2d[0].pos.x, 3.0);
}
