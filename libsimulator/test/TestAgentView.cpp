// SPDX-License-Identifier: LGPL-3.0-or-later
#include "AgentView.hpp"
#include "EnvironmentQuery.hpp"
#include "GenericAgent.hpp"
#include "GeometryBuilder.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModels/CollisionFreeSpeedModel/CollisionFreeSpeedModel.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace
{
using State = CollisionFreeSpeedModel::State;

GenericAgent MakeAgent(Point pos, double radius = 0.2)
{
    State s{};
    s.radius = radius;
    return GenericAgent(
        GenericAgent::ID::Invalid,
        jps::UniqueID<Journey>::Invalid,
        jps::UniqueID<BaseStage>::Invalid,
        pos,
        std::move(s));
}

Geometry2D OpenGeometry()
{
    GeometryBuilder b{};
    b.AddAccessibleArea({{-100, -100}, {100, -100}, {100, 100}, {-100, 100}});
    return b.Build();
}

// Geometry with a thin wall at x≈1 that blocks line-of-sight across it.
Geometry2D WalledGeometry()
{
    GeometryBuilder b{};
    b.AddAccessibleArea({{-100, -100}, {100, -100}, {100, 100}, {-100, 100}});
    b.ExcludeFromAccessibleArea({{0.9, -50}, {1.1, -50}, {1.1, 50}, {0.9, 50}});
    return b.Build();
}

struct Environment {
    AgentContainer<GenericAgent> agents{};
    NeighborhoodSearch<GenericAgent> neighborhood_search{5.0};

    void add_agent(Point pos, double radius = 0.2) { agents.push_back(MakeAgent(pos, radius)); }

    EnvironmentQuery query(const Geometry2D& geo)
    {
        neighborhood_search.Update(agents);
        return {geo, neighborhood_search};
    }

    // The first agent added is the one every test queries from.
    AgentView first_agent_view(const EnvironmentQuery& q) const { return {q, agents[0]}; }
};
} // namespace

TEST(AgentView, OtherAgentsInRangeExcludesSelf)
{
    Environment env{};
    env.add_agent({0, 0});
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(100.0);
    EXPECT_TRUE(result.empty());
}

TEST(AgentView, OtherAgentsInRangeNoFilterReturnsAllInRadius)
{
    Environment env{};
    env.add_agent({0, 0}); // querying agent
    env.add_agent({1, 0});
    env.add_agent({0, 1});
    env.add_agent({-1, 0});
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(5.0);
    EXPECT_EQ(result.size(), 3u);
}

TEST(AgentView, OtherAgentsInRangeCustomFilterRejectsAll)
{
    Environment env{};
    env.add_agent({0, 0});
    env.add_agent({1, 0});
    env.add_agent({0, 1});
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(
        5.0, [](const NeighborView&) { return false; });
    EXPECT_TRUE(result.empty());
}

TEST(AgentView, OtherAgentsInRangeCustomFilterSelectsSubset)
{
    Environment env{};
    env.add_agent({0, 0}); // querying agent
    env.add_agent({1, 0}); // positive x — kept
    env.add_agent({0, 1}); // positive y — kept
    env.add_agent({-1, 0}); // negative x — filtered out
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(
        5.0, [](const NeighborView& n) { return n.relative_position.x >= 0.0; });

    ASSERT_EQ(result.size(), 2u);
    for(const auto& neighbor : result) {
        EXPECT_GE(neighbor.relative_position.x, 0.0);
    }
}

TEST(AgentView, NoGeometryBetweenFiltersOccludedAgents)
{
    Environment env{};
    env.add_agent({0, 0}); // querying agent
    env.add_agent({2, 0}); // behind wall — occluded
    env.add_agent({0, 1}); // same side as querying agent — visible
    const auto geo = WalledGeometry();
    const auto q = env.query(geo);

    const auto view = env.first_agent_view(q);
    const auto result = view.other_agents_in_range(
        5.0, [&](const NeighborView& n) { return view.no_geometry_between(n.relative_position); });

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].relative_position, Point(0, 1));
}

TEST(AgentView, OtherAgentsInRangeCustomFilterReceivesNoSelf)
{
    // Verify the filter is never called with the querying agent itself.
    Environment env{};
    env.add_agent({0, 0});
    env.add_agent({1, 0});
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    int calls = 0;
    env.first_agent_view(q).other_agents_in_range(5.0, [&](const NeighborView&) {
        ++calls;
        return true;
    });
    EXPECT_EQ(calls, 1);
}

TEST(AgentView, OtherAgentsInRangeOutOfRadiusNotReturned)
{
    Environment env{};
    env.add_agent({0, 0});
    env.add_agent({50, 0}); // far away
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(
        1.0, [](const NeighborView&) { return true; });
    EXPECT_TRUE(result.empty());
}

TEST(AgentView, AgentsOnTheSamePositionSeeEachOther)
{
    Environment env{};
    env.add_agent({3, 4});
    env.add_agent({3, 4});
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(1.0);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].state, &env.agents[1].model);
}

// Every other test here queries from {0,0} and would not notice a centre that
// silently defaults to the origin.
TEST(AgentView, OtherAgentsInRangeCentresOnTheQueryingAgentNotTheOrigin)
{
    Environment env{};
    env.add_agent({50, 30}); // querying agent, deliberately away from the origin
    env.add_agent({51, 30}); // its actual neighbor
    env.add_agent({0.5, 0}); // decoy next to the origin
    const auto geo = OpenGeometry();
    const auto q = env.query(geo);

    const auto result = env.first_agent_view(q).other_agents_in_range(2.0);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].relative_position, Point(1, 0));
}
