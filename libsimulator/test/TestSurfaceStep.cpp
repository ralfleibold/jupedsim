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

/// A two-storey geometry as ONE connected mesh: ground floor (x in [0,16],
/// y in [0,6], z=0, split into two quads at x=12) -> narrow ramp (x in
/// [12,16], climbing y 6..12 to z=3) -> upper floor in a U back over the
/// ground: two quads forward/left (y in [12,16]) and a final one (x in
/// [6,12], y in [0,12]) directly above the ground floor. That (x,y) overlap
/// forces a region split whose seam lies on a flat, continuous plane --
/// either between the two ground quads or between the last two upper quads,
/// depending on the split's growth order.
SurfaceMesh upper_floor_over_ground()
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

/// Region id of the sheet at height @p z covering the agent position.
std::size_t region_at_height(const Geometry3D& geo, const GenericAgent& agent, double z)
{
    for(std::size_t region = 0; region < geo.region_count(); ++region) {
        const auto loc = geo.locate_in_region(region, {agent.pos.x, agent.pos.y});
        if(loc.face != SurfaceMesh::null_face() && std::abs(loc.point.z() - z) < 1e-9) {
            return region;
        }
    }
    ADD_FAILURE() << "no sheet at (" << agent.pos.x << ", " << agent.pos.y << ", z=" << z << ")";
    return 0;
}

/// The agent's current on-surface height (z of its anchor).
double height_of(const Geometry3D& geo, const GenericAgent& agent)
{
    const auto loc = geo.locate_in_region(agent.regionId, {agent.pos.x, agent.pos.y});
    EXPECT_NE(loc.face, SurfaceMesh::null_face());
    return loc.point.z();
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
    agents.front().regionId = region_at_height(geo, agents.front(), 0.4 * 2);

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < 60; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents);
    }

    // v0=1.2 for 3 s moves the agent well up the ramp on a straight line.
    const auto& agent = agents.front();
    EXPECT_GT(agent.pos.y, 4.0);
    EXPECT_NEAR(agent.pos.x, 10.0, 1e-6);
    // The anchor tracks the agent, on the ramp plane z = 0.4*y, region 0.
    EXPECT_NEAR(height_of(geo, agent), 0.4 * agent.pos.y, 1e-9);
    EXPECT_EQ(agent.regionId, 0u);
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
    agents[0].regionId = region_at_height(geo, agents[0], 0);
    agents[1].regionId = region_at_height(geo, agents[1], 3.0);

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < 60; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents);
    }

    EXPECT_GT(agents[0].pos.x, 6.5);
    EXPECT_NEAR(agents[0].pos.y, 5.0, 1e-6);
    EXPECT_NEAR(height_of(geo, agents[0]), 0.0, 1e-9);
    EXPECT_NEAR(height_of(geo, agents[1]), 3.0, 1e-9);
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
    agents3d[0].regionId = region_at_height(geo, agents3d[0], 0);
    agents3d[1].regionId = region_at_height(geo, agents3d[1], 0);

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < steps; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents3d);
    }

    for(std::size_t i = 0; i < agents2d.size(); ++i) {
        EXPECT_NEAR(agents3d[i].pos.x, agents2d[i].pos.x, 1e-9) << "agent " << i;
        EXPECT_NEAR(agents3d[i].pos.y, agents2d[i].pos.y, 1e-9) << "agent " << i;
    }
    // Sanity: they actually moved.
    EXPECT_GT(agents2d[0].pos.x, 3.0);
}

TEST(SurfaceStep, LiftedHoleGeometryReproduces2DPipelineWithWallContact)
{
    // Both pipelines run on the SAME 2D input: the 2D one on the
    // CollisionGeometry, the 3D one on its polygon lifted to z=0 via
    // initialize_from_polygon. Agent 0 aims straight through the hole and must
    // stall against its wall; agent 1 squeezes past just below it -- so wall
    // interaction shapes both trajectories, and they must match exactly.
    const auto goal_a = Point{8, 5};
    const auto goal_b = Point{8, 3.5};
    constexpr int steps = 100;

    GeometryBuilder builder{};
    builder.AddAccessibleArea({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    builder.ExcludeFromAccessibleArea({{4, 4}, {6, 4}, {6, 6}, {4, 6}});
    const auto geometry2d = builder.Build();

    AgentContainer<GenericAgent> agents2d{};
    agents2d.push_back(make_agent({2, 5}, goal_a));
    agents2d.push_back(make_agent({2, 3.5}, goal_b));

    OperationalDecisionSystem system{std::make_unique<CollisionFreeSpeedModel>(default_cfsm())};
    NeighborhoodSearch<GenericAgent> neighborhoodSearch{2.2};
    for(int step = 0; step < steps; ++step) {
        neighborhoodSearch.Update(agents2d);
        system.Run(dT, 0., neighborhoodSearch, geometry2d, agents2d);
    }

    Geometry3D geo{};
    geo.initialize_from_polygon(geometry2d.Polygon());
    const auto model = default_cfsm();
    AgentContainer<GenericAgent> agents3d{};
    agents3d.push_back(make_agent({2, 5}, goal_a));
    agents3d.push_back(make_agent({2, 3.5}, goal_b));
    agents3d[0].regionId = region_at_height(geo, agents3d[0], 0);
    agents3d[1].regionId = region_at_height(geo, agents3d[1], 0);

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    for(int step = 0; step < steps; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents3d);
    }

    for(std::size_t i = 0; i < agents2d.size(); ++i) {
        EXPECT_NEAR(agents3d[i].pos.x, agents2d[i].pos.x, 1e-9) << "agent " << i;
        EXPECT_NEAR(agents3d[i].pos.y, agents2d[i].pos.y, 1e-9) << "agent " << i;
    }
    // Sanity on the shared outcome: agent 0 is held up by the hole's wall,
    // agent 1 walked past below it.
    EXPECT_LT(agents2d[0].pos.x, 4.0);
    EXPECT_GT(agents2d[1].pos.x, 5.0);
}

TEST(SurfaceStep, SeamCrossingFlipsRegionWithoutDisturbingTheWalk)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(upper_floor_over_ground());
    ASSERT_EQ(geo.region_count(), 2);

    const auto region_at = [&geo](double x, double y, double z) {
        const auto loc = geo.face_below(Point3D{x, y, z + 1.0});
        EXPECT_NE(loc.face, SurfaceMesh::null_face());
        EXPECT_NEAR(loc.point.z(), z, 1e-9);
        return geo.region_of(loc.face);
    };

    // The split puts the seam on exactly one of the two flat quad pairs; find
    // out which, then walk straight across it.
    const bool seam_on_upper = region_at(9, 15, 3) != region_at(9, 2, 3);
    const bool seam_on_ground = region_at(3, 3, 0) != region_at(15, 3, 0);
    ASSERT_NE(seam_on_upper, seam_on_ground);

    const Point start = seam_on_upper ? Point{9, 15} : Point{3, 3};
    const Point destination = seam_on_upper ? Point{9, 2} : Point{15, 3};
    const double z = seam_on_upper ? 3.0 : 0.0;

    const auto model = default_cfsm();
    AgentContainer<GenericAgent> agents{};
    agents.push_back(make_agent(start, destination));
    agents.front().regionId = region_at_height(geo, agents.front(), z);

    InformationGatherer3D gatherer{geo, 2.2, 2.2};
    auto region = agents.front().regionId;
    const auto start_region = region;
    int region_flips = 0;
    for(int step = 0; step < 260; ++step) {
        run_surface_step(dT, model, gatherer, geo, agents);
        // The seam is no wall and no crease: the walk stays a straight line on
        // a constant height; only the region label may change.
        if(seam_on_upper) {
            ASSERT_NEAR(agents.front().pos.x, start.x, 1e-6) << "step " << step;
        } else {
            ASSERT_NEAR(agents.front().pos.y, start.y, 1e-6) << "step " << step;
        }
        ASSERT_NEAR(height_of(geo, agents.front()), z, 1e-9) << "step " << step;
        if(const auto current = agents.front().regionId; current != region) {
            ++region_flips;
            region = current;
        }
    }
    EXPECT_EQ(region_flips, 1);
    EXPECT_NE(region, start_region);
    // The agent actually reached the far side of the seam.
    EXPECT_NEAR(agents.front().pos.x, destination.x, 0.2);
    EXPECT_NEAR(agents.front().pos.y, destination.y, 0.2);
}
