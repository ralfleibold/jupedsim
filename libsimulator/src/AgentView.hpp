// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "EnvironmentQuery.hpp"
#include "GenericAgent.hpp"
#include "LineSegment.hpp"
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

/// A wall segment as seen from the agent that asked for it. It carries what agents typically
/// use/compute to avoid repetitions plus harmonizes computation.
struct WallView {
    /// The segment itself, relative to the agent.
    LineSegment segment;
    /// The point on the segment closest to the agent.
    Point closest_point;
    /// Distance to that point.
    double distance;
    /// Unit vector pointing from the wall towards the agent, i.e. the direction a repulsion
    /// acts in. Zero for an agent standing exactly on the wall, where no direction exists.
    Point normal;
};

/// Substitutes the model state neighbors are seen with.
///
/// A model that delegates a step to another model hands over a view of the world, but the
/// delegate expects neighbors to carry its own state type. An implementation maps each
/// neighbor's stored state to one the delegate understands and owns the mapped states for as
/// long as it lives.
class NeighborStates
{
public:
    virtual ~NeighborStates() = default;
    virtual const OperationalModelState& state_of(const GenericAgent& agent) const = 0;
};

/// What an agent perceives of its surroundings, expressed relative to where it
/// stands. Agents do not know absolute positions as they do not need to.
class AgentView
{
public:
    AgentView(
        const EnvironmentQuery& world,
        const GenericAgent& agent,
        const NeighborStates* neighborStates = nullptr)
        : _world(world), _agent(agent), _neighborStates(neighborStates)
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
            const NeighborView neighbor{
                candidate.position - _agent.position,
                _neighborStates ? &_neighborStates->state_of(candidate) : &candidate.model};
            if(filter(neighbor)) {
                neighbors.push_back(neighbor);
            }
        });
        return neighbors;
    }

    /// Whether neighbors are seen through a substituted state rather than their own.
    bool has_neighbor_states() const { return _neighborStates != nullptr; }

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
    /// The segments as seen from the agent. Lazy range, no copies.
    /// Must stay above walls_nearby() and walls_in_range() in code: an 'auto' return type is
    /// deduced from the body, so unlike other members this one cannot be called before it is
    /// defined.
    auto as_seen_from_agent(Geometry2D::LineSegmentRange segments) const
    {
        return segments | std::views::transform([origin = _agent.position](const LineSegment& s) {
                   const LineSegment segment{s.p1 - origin, s.p2 - origin};
                   const Point closest_point = segment.ShortestPoint(Point{});
                   return WallView{
                       .segment = segment,
                       .closest_point = closest_point,
                       .distance = closest_point.Norm(),
                       .normal = (Point{} - closest_point).Normalized()};
               });
    }

public:
    /// Walls in the grid cells around the agent. Which ones are returned depends on the
    /// underlying grid cell size.
    /// Returned as lazy range.
    auto walls_nearby() const
    {
        return as_seen_from_agent(_world.LineSegmentsInRange(_agent.position));
    }

    /// Walls within 'distance' of the agent. Returns lazy range.
    auto walls_in_range(double distance) const
    {
        return as_seen_from_agent(_world.LineSegmentsInRange(_agent.position, distance));
    }

protected:
    const EnvironmentQuery& _world;
    const GenericAgent& _agent;
    /// Null in the common case, where neighbors are seen with their own state.
    const NeighborStates* _neighborStates;
};

/// An AgentView plus what only holds for one step (dT + next target).
class AgentStep : public AgentView
{
public:
    AgentStep(
        const EnvironmentQuery& world,
        const GenericAgent& agent,
        double dt,
        const NeighborStates* neighborStates = nullptr)
        : AgentView(world, agent, neighborStates), _dt(dt)
    {
    }

    /// The same step, but with neighbors seen through 'states'. 'states' has to outlive the
    /// returned step.
    AgentStep with_neighbor_states(const NeighborStates& states) const
    {
        return AgentStep{_world, _agent, _dt, &states};
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
