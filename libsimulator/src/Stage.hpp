// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "GeometricFunctions.hpp"
#include "InformationForUpdate.hpp"
#include "LineSegment.hpp"
#include "NeighborhoodSearch.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "UniqueID.hpp"
#include "Util.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <limits>
#include <set>
#include <unordered_set>
#include <variant>
#include <vector>

class Simulation;

class BaseStage;

/// Stage updates ask the world one question: "who and which walls are around
/// this slot?" -- expressed as a QueryAt callable
/// `(Point, double radius, double z) -> InformationForUpdate`. @p z is the
/// stage's anchored height; the surface adapter uses it to pick the sheet,
/// this 2D adapter ignores it.
/// The returned info's walls may point into storage reused by the next call.
inline auto stage_query_2d(
    const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
    const CollisionGeometry& geometry)
{
    return [&neighborhoodSearch, &geometry](Point p, double radius, double) {
        InformationForUpdate info{};
        info.neighbors = neighborhoodSearch.GetNeighboringAgents(p, radius);
        const auto& walls = geometry.LineSegmentsInApproxDistanceTo(p);
        info.walls = std::span<const LineSegment>(walls.data(), walls.size());
        return info;
    };
}

enum class WaitingSetState {
    Active,
    Inactive,
};

class BaseProxy
{
protected:
    Simulation* simulation;
    BaseStage* stage;

    BaseProxy(Simulation* simulation_, BaseStage* stage_) : simulation(simulation_), stage(stage_)
    {
    }
    virtual ~BaseProxy() = default;

public:
    size_t CountTargeting() const;
};

class WaypointProxy : public BaseProxy
{
public:
    WaypointProxy(Simulation* simulation_, BaseStage* stage_) : BaseProxy(simulation_, stage_) {}
};

class NotifiableWaitingSetProxy : public BaseProxy
{
public:
    NotifiableWaitingSetProxy(Simulation* simulation_, BaseStage* stage_)
        : BaseProxy(simulation_, stage_)
    {
    }
    void State(WaitingSetState newState);
    WaitingSetState State() const;
    size_t CountWaiting() const;
    const std::vector<GenericAgent::ID>& Waiting() const;
};

class NotifiableQueueProxy : public BaseProxy
{
public:
    NotifiableQueueProxy(Simulation* simulation_, BaseStage* stage_)
        : BaseProxy(simulation_, stage_)
    {
    }

    size_t CountEnqueued() const;
    const std::vector<GenericAgent::ID>& Enqueued() const;
    void Pop(size_t count);
};

class ExitProxy : public BaseProxy
{
public:
    ExitProxy(Simulation* simulation_, BaseStage* stage_) : BaseProxy(simulation_, stage_) {}
};

class DirectSteeringProxy : public BaseProxy
{
public:
    DirectSteeringProxy(Simulation* simulation_, BaseStage* stage_) : BaseProxy(simulation_, stage_)
    {
    }
};

using StageProxy = std::variant<
    WaypointProxy,
    NotifiableWaitingSetProxy,
    NotifiableQueueProxy,
    ExitProxy,
    DirectSteeringProxy>;

class BaseStage
{
public:
    using ID = jps::UniqueID<BaseStage>;

protected:
    ID id;
    size_t targeting{0};
    /// Anchored surface height of the stage's representative point (set by
    /// Simulation::AddStage; 0 on the 2D path and on flat lifts). Reached
    /// checks stay 2D but add a z-band around this height, so a stage never
    /// triggers for agents on a floor above or below it.
    double _z{0};

public:
    virtual ~BaseStage() = default;
    virtual bool IsCompleted(const GenericAgent& agent) = 0;
    virtual Point Target(const GenericAgent& agent) = 0;
    virtual StageProxy Proxy(Simulation* simulation_) = 0;
    ID Id() const { return id; }
    void set_z(double z) { _z = z; }
    double z() const { return _z; }
    size_t CountTargeting() const { return targeting; }
    void IncreaseTargeting() { targeting = targeting + 1; }
    void DecreaseTargeting()
    {
        assert(targeting >= 1);
        targeting = targeting - 1;
    }
};

template <>
struct fmt::formatter<BaseStage> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const BaseStage& s, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "(id={}, targeting={})", s.Id(), s.CountTargeting());
    }
};

class Waypoint : public BaseStage
{
    Point position;
    double distance;

public:
    Waypoint(Point position_, double distance_);
    ~Waypoint() override = default;
    bool IsCompleted(const GenericAgent& agent) override;
    Point Target(const GenericAgent& agent) override;
    StageProxy Proxy(Simulation* simulation_) override;
    Point Position() const { return position; };
};

/// Notifies simulation of all agents that need to be removed at the beginning of the next iteration
class Exit : public BaseStage
{
    Polygon area;
    std::vector<GenericAgent::ID>& toRemove;

public:
    Exit(Polygon area, std::vector<GenericAgent::ID>& toRemove_);
    ~Exit() override = default;
    bool IsCompleted(const GenericAgent& agent) override;
    Point Target(const GenericAgent& agent) override;
    StageProxy Proxy(Simulation* simulation_) override;
    Polygon Position() const { return area; };
};

class NotifiableWaitingSet : public BaseStage
{
    std::vector<Point> slots;
    std::vector<GenericAgent::ID> occupants{};
    WaitingSetState state{WaitingSetState::Active};

public:
    NotifiableWaitingSet(std::vector<Point> slots_);
    ~NotifiableWaitingSet() override = default;
    bool IsCompleted(const GenericAgent& agent) override;
    Point Target(const GenericAgent& agent) override;
    StageProxy Proxy(Simulation* simulation_) override;
    void State(WaitingSetState s);
    WaitingSetState State() const;
    template <typename QueryAt>
    void Update(QueryAt&& queryAt);
    const std::vector<GenericAgent::ID>& Occupants() const;
    const std::vector<Point>& Slots() const { return slots; };
};

/// Among @p info gathered at @p slot_pos, the id of the closest agent that is
/// eligible (per @p isEligible) and visible from the slot (no wall segment
/// between them); Invalid if there is none. Shared slot-occupancy core of the
/// waiting set and the queue.
template <typename Pred>
GenericAgent::ID
closest_visible_candidate(Point slot_pos, const InformationForUpdate& info, Pred&& isEligible)
{
    GenericAgent::ID occupant = GenericAgent::ID::Invalid;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& agent : info.neighbors) {
        if(!isEligible(agent)) {
            continue;
        }
        const auto slot_to_agent = LineSegment(slot_pos, agent.pos);
        const auto blocked =
            std::any_of(info.walls.begin(), info.walls.end(), [&slot_to_agent](const auto& wall) {
                return intersects(slot_to_agent, wall);
            });
        if(blocked) {
            continue;
        }
        const auto distance = (agent.pos - slot_pos).Norm();
        if(distance < min_distance) {
            min_distance = distance;
            occupant = agent.id;
        }
    }
    return occupant;
}

template <typename QueryAt>
void NotifiableWaitingSet::Update(QueryAt&& queryAt)
{
    if(state == WaitingSetState::Inactive) {
        return;
    }
    const auto count_occupants = occupants.size();
    if(count_occupants == slots.size()) {
        return;
    }

    for(size_t index = count_occupants; index < slots.size(); ++index) {
        const auto slot_pos = slots[index];
        const auto occupant = closest_visible_candidate(
            slot_pos, queryAt(slot_pos, 2., _z), [this](const GenericAgent& agent) {
                return agent.stageId == id &&
                       std::find(std::begin(occupants), std::end(occupants), agent.id) ==
                           std::end(occupants);
            });
        if(occupant != GenericAgent::ID::Invalid) {
            occupants.push_back(occupant);
        } else {
            return;
        }
    }
}

class NotifiableQueue : public BaseStage
{

private:
    std::vector<Point> slots;
    std::vector<GenericAgent::ID> occupants{};
    std::set<GenericAgent::ID> exitingThisUpdate{};

public:
    NotifiableQueue(std::vector<Point> slots_);
    ~NotifiableQueue() override = default;
    bool IsCompleted(const GenericAgent& agent) override;
    Point Target(const GenericAgent& agent) override;
    StageProxy Proxy(Simulation* simulation_) override;
    template <typename QueryAt>
    void Update(QueryAt&& queryAt);
    void Pop(size_t count);
    const std::vector<GenericAgent::ID>& Occupants() const;
    const std::vector<Point>& Slots() const { return slots; };
};

template <typename QueryAt>
void NotifiableQueue::Update(QueryAt&& queryAt)
{
    const auto count_occupants = occupants.size();
    if(count_occupants == slots.size()) {
        return;
    }

    for(size_t index = count_occupants; index < slots.size(); ++index) {
        const auto slot_pos = slots[index];
        const auto occupant = closest_visible_candidate(
            slot_pos, queryAt(slot_pos, 2., _z), [this](const GenericAgent& agent) {
                return agent.stageId == id && !Contains(occupants, agent.id) &&
                       !exitingThisUpdate.contains(agent.id);
            });
        if(occupant != GenericAgent::ID::Invalid) {
            occupants.emplace_back(occupant);
        } else {
            return;
        }
    }
}

class DirectSteering : public BaseStage
{
public:
    DirectSteering() = default;
    ~DirectSteering() override = default;
    bool IsCompleted(const GenericAgent&) override { return false; };
    Point Target(const GenericAgent& agent) override { return agent.target; };
    StageProxy Proxy(Simulation* simulation) override
    {
        return DirectSteeringProxy(simulation, this);
    };
};
