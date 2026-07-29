// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "EnvironmentQuery.hpp"
#include "GenericAgent.hpp"
#include "Point.hpp"

#include <concepts>
#include <ranges>
#include <type_traits>
#include <vector>

/// A neighbouring agent as seen from the agent that asked for it.
struct NeighborView {
    Point relative_position;
    const OperationalModelState* state;
};

/// What an agent perceives of its surroundings, expressed relative to where it
/// stands. Agents do not know absolute positions as they do not need to.
class AgentView
{
public:
    AgentView(const EnvironmentQuery& world, const GenericAgent& agent)
        : _world(world), _agent(agent)
    {
    }

    struct AcceptAllNeighbors {
        bool operator()(const NeighborView&) const { return true; }
    };

    /// All agents within 'radius', excluding this agent. 'filter' is never called with it.
    template <std::predicate<const NeighborView&> Pred = AcceptAllNeighbors>
    std::vector<NeighborView> other_agents_in_range(double radius, Pred filter = {}) const
    {
        std::vector<NeighborView> neighbors{};
        _world.for_each_agent_in_range(_agent.position, radius, [&](const GenericAgent& candidate) {
            if(candidate.id == _agent.id) {
                return;
            }
            const NeighborView neighbor{candidate.position - _agent.position, &candidate.model};
            if(filter(neighbor)) {
                neighbors.push_back(neighbor);
            }
        });
        return neighbors;
    }

    /// Whether the straight line to a point at 'relative_position' is free of geometry.
    bool no_geometry_between(Point relative_position) const
    {
        return _world.NoGeometryBetween(_agent.position, _agent.position + relative_position);
    }

    /// Whether the point reached by moving 'relative_position' is inside the walkable area.
    bool inside_geometry(Point relative_position) const
    {
        return _world.InsideGeometry(_agent.position + relative_position);
    }

private:
    /// The segments as relative ones. Lazy range, no copies.
    auto relative(Geometry2D::LineSegmentRange segments) const
    {
        return segments | std::views::transform([origin = _agent.position](const LineSegment& s) {
                   return LineSegment{s.p1 - origin, s.p2 - origin};
               });
    }

public:
    /// Wall segments in the grid cells around the agent, relative to it. The returned segments
    /// depend on the underlying grid cell size.
    /// Returned as lazy range.
    auto walls_nearby() const { return relative(_world.LineSegmentsInRange(_agent.position)); }

    /// Wall segments within 'distance' of the agent, relative to it. Returns lazy range.
    auto walls_in_range(double distance) const
    {
        return relative(_world.LineSegmentsInRange(_agent.position, distance));
    }

protected:
    const EnvironmentQuery& _world;
    const GenericAgent& _agent;
};

/// An AgentView plus what only holds for one step (dT + next target).
class AgentStep : public AgentView
{
public:
    AgentStep(const EnvironmentQuery& world, const GenericAgent& agent, double dt)
        : AgentView(world, agent), _dt(dt)
    {
    }

    double dt() const { return _dt; }

    Point to_next_target() const { return _agent.nextTarget - _agent.position; }

private:
    double _dt;
};

// Both views are passed by reference and never deleted through a base pointer; keeping
// them non-polymorphic keeps them free of a vtable and fully inlinable.
static_assert(!std::is_polymorphic_v<AgentView>);
static_assert(!std::is_polymorphic_v<AgentStep>);
