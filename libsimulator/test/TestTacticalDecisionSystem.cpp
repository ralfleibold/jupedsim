// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "SimulationError.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"
#include "TacticalDecisionSystem.hpp"

#include <gtest/gtest.h>

namespace
{
/// A flat 10x10 room at z=0 continuing into a ramp that rises to z=4 at y=20,
/// welded along y=10: one region with real height variation.
SurfaceMesh flat_into_ramp()
{
    SurfaceMesh mesh{};
    const auto a = mesh.add_vertex(Point3D{0, 0, 0});
    const auto b = mesh.add_vertex(Point3D{10, 0, 0});
    const auto c = mesh.add_vertex(Point3D{10, 10, 0});
    const auto d = mesh.add_vertex(Point3D{0, 10, 0});
    const auto e = mesh.add_vertex(Point3D{10, 20, 4});
    const auto f = mesh.add_vertex(Point3D{0, 20, 4});
    mesh.add_face(a, b, c);
    mesh.add_face(a, c, d);
    mesh.add_face(d, c, e);
    mesh.add_face(d, e, f);
    return mesh;
}

GenericAgent agent_at(Point pos, Point target)
{
    GenericAgent agent(
        GenericAgent::ID::Invalid,
        jps::UniqueID<Journey>::Invalid,
        jps::UniqueID<BaseStage>::Invalid,
        pos,
        CollisionFreeSpeedModelData{});
    agent.target = target;
    return agent;
}
} // namespace

TEST(TacticalDecisionSystem, SurfaceModeTakesTheSourceHeightFromTheRegionAnchor)
{
    Geometry3D geo{};
    geo.initialize_from_mesh(flat_into_ramp());
    ASSERT_EQ(geo.region_count(), 1);
    SurfaceMeshShortestPathRoutingEngine engine{geo};

    // On the ramp at (5,16) the surface sits at z=2.4: the routing query point
    // must carry that height, a z=0 point projects past the surface and cannot
    // route from there (which is exactly what the legacy 2D path does -- fine
    // on flat lifts, wrong here).
    AgentContainer<GenericAgent> agents{};
    agents.push_back(agent_at({5, 16}, {5, 5}));
    const TacticalDecisionSystem tactical{};

    EXPECT_THROW(tactical.Run(engine, agents, nullptr), SimulationError);

    // With the surface, the source anchors at (5,16,2.4) and the geodesic down
    // to (5,5,0) runs straight along x=5.
    tactical.Run(engine, agents, &geo);
    const auto& destination = agents.front().destination;
    EXPECT_NEAR(destination.x, 5.0, 1e-9);
    EXPECT_GT(destination.y, 5.0 - 1e-9);
    EXPECT_LT(destination.y, 16.0);
}
