// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Simulation.hpp"

#include "CollisionGeometry.hpp"
#include "GeneralizedCentrifugalForceModelData.hpp"
#include "GenericAgent.hpp"
#include "IteratorPair.hpp"
#include "Journey.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "SimulationClock.hpp"
#include "SimulationError.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "Tracing.hpp"
#include "Visitor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

Simulation::Simulation(
    std::unique_ptr<OperationalModel>&& operationalModel,
    std::unique_ptr<Geometry3D>&& geometry,
    std::unique_ptr<RoutingEngine3D>&& routingEngine,
    double dT,
    bool runIn2d)
    : _clock(dT)
    , _operationalDecisionSystem(std::move(operationalModel))
    , _geometry3d(std::move(geometry))
    , _routingEngine(std::move(routingEngine))
{
    if(!_geometry3d) {
        throw SimulationError("Simulation requires a geometry.");
    }
    if(!_routingEngine) {
        throw SimulationError("Simulation requires a routing engine.");
    }
    _geometry = _geometry3d->collision_geometry();
    if(!_geometry) {
        throw SimulationError(
            "The geometry has no projected 2D view (it was not built from a polygon); "
            "the simulation still requires one for collision handling.");
    }
    if(!runIn2d) {
        // Cell size matches _neighborhoodSearch{2.2} for gather parity; the
        // vertical interaction band is a placeholder until it becomes
        // configurable (flat lifts never exercise it).
        _gatherer3d = std::make_unique<InformationGatherer3D>(*_geometry3d, 2.2, 2.2);
    }
}

const SimulationClock& Simulation::Clock() const
{
    return _clock;
}

void Simulation::SetTracing(bool status)
{
    if(status) {
        Profiler::instance().enable();
    } else {
        Profiler::instance().disable();
    }
};

void Simulation::Iterate()
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Total Iteration", General);

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agent Removal System", Detailed);
        _agentRemovalSystem.Run(_agents, _removedAgentsInLastIteration, _stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Neighborhood Search", Detailed);
        if(_gatherer3d) {
            // Re-anchor after agent removal; run_surface_step consumes this
            // index and refreshes it once more after the agents moved.
            _gatherer3d->update(*_geometry3d, _agents);
        } else {
            _neighborhoodSearch.Update(_agents);
        }
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Stage System", Detailed);
        if(_gatherer3d) {
            _stageSystem.RunOnSurface(_stageManager, *_gatherer3d, *_geometry3d);
        } else {
            _stageSystem.Run(_stageManager, _neighborhoodSearch, *_geometry);
        }
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Strategical Decision System", General);
        _stategicalDecisionSystem.Run(_journeys, _agents, _stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Tactical Decision System", General);
        _tacticalDecisionSystem.Run(
            *_routingEngine, _agents, _gatherer3d ? _geometry3d.get() : nullptr);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Operational Decision System", General);
        if(_gatherer3d) {
            _operationalDecisionSystem.RunOnSurface(
                _clock.dT(), *_gatherer3d, *_geometry3d, _agents);
        } else {
            _operationalDecisionSystem.Run(
                _clock.dT(), _clock.ElapsedTime(), _neighborhoodSearch, *_geometry, _agents);
        }
    }
    _clock.Advance();
}

Journey::ID Simulation::AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Journey", Detailed);
    std::map<BaseStage::ID, JourneyNode> nodes;
    bool containsDirectSteering =
        std::find_if(std::begin(stages), std::end(stages), [this](auto const& pair) {
            return std::holds_alternative<DirectSteeringProxy>(Stage(pair.first));
        }) != std::end(stages);

    if(containsDirectSteering && stages.size() > 1) {
        throw SimulationError(
            "Journeys containing a DirectSteeringStage, may only contain this stage.");
    }

    std::transform(
        std::begin(stages),
        std::end(stages),
        std::inserter(nodes, std::end(nodes)),
        [this](auto const& pair) -> std::pair<BaseStage::ID, JourneyNode> {
            const auto& [id, desc] = pair;
            auto stage = _stageManager.Stage(id);
            return {
                id,
                JourneyNode{
                    stage,
                    std::visit(
                        overloaded{
                            [stage](
                                const NonTransitionDescription&) -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(stage);
                            },
                            [this](const FixedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(
                                    _stageManager.Stage(d.NextId()));
                            },
                            [this](const RoundRobinTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<std::tuple<BaseStage*, uint64_t>> weightedStages{};
                                weightedStages.reserve(d.WeightedStages().size());

                                std::transform(
                                    std::begin(d.WeightedStages()),
                                    std::end(d.WeightedStages()),
                                    std::back_inserter(weightedStages),
                                    [this](auto const& pair) -> std::tuple<BaseStage*, uint64_t> {
                                        const auto& [id, weight] = pair;
                                        return {_stageManager.Stage(id), weight};
                                    });

                                return std::make_unique<RoundRobinTransition>(weightedStages);
                            },
                            [this](const LeastTargetedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<BaseStage*> candidates{};
                                candidates.reserve(d.TargetCandidates().size());

                                std::transform(
                                    std::begin(d.TargetCandidates()),
                                    std::end(d.TargetCandidates()),
                                    std::back_inserter(candidates),
                                    [this](auto const& id) -> BaseStage* {
                                        return _stageManager.Stage(id);
                                    });

                                return std::make_unique<LeastTargetedTransition>(candidates);
                            }},
                        desc)}};
        });

    auto journey = std::make_unique<Journey>(std::move(nodes));
    const auto id = journey->Id();
    _journeys.emplace(id, std::move(journey));
    return id;
}

BaseStage::ID Simulation::AddStage(const StageDescription stageDescription)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Stage", Detailed);
    // Stages carry no region yet, so surface mode anchors in region 0 -- the
    // same interim choice StageSystem::RunOnSurface makes for slot queries
    // (per-region stage anchoring is a C8 decision).
    const auto inside = [this](Point p) {
        if(_gatherer3d) {
            return _geometry3d->locate_in_region(0, {p.x, p.y}).face != SurfaceMesh::null_face();
        }
        return _geometry->InsideGeometry(p);
    };
    std::visit(
        overloaded{
            [&inside](const WaypointDescription& d) -> void {
                if(!inside(d.position)) {
                    throw SimulationError("WayPoint {} not inside walkable area", d.position);
                }
            },
            [&inside](const ExitDescription& d) -> void {
                if(!inside(d.polygon.Centroid())) {
                    throw SimulationError("Exit {} not inside walkable area", d.polygon.Centroid());
                }
            },
            [&inside](const NotifiableWaitingSetDescription& d) -> void {
                for(const auto& point : d.slots) {
                    if(!inside(point)) {
                        throw SimulationError(
                            "NotifiableWaitingSet point {} not inside walkable area", point);
                    }
                }
            },
            [&inside](const NotifiableQueueDescription& d) -> void {
                for(const auto& point : d.slots) {
                    if(!inside(point)) {
                        throw SimulationError(
                            "NotifiableQueue point {} not inside walkable area", point);
                    }
                }
            },
            [](const DirectSteeringDescription&) -> void {

            }},
        stageDescription);

    return _stageManager.AddStage(stageDescription, _removedAgentsInLastIteration);
}

GenericAgent::ID Simulation::AddAgent(GenericAgent agent)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Agent", Detailed);
    // The anchor of the new agent on the surface; only set in surface mode.
    Geometry3D::FaceLocation anchor{SurfaceMesh::null_face(), {}};
    if(_gatherer3d) {
        auto num_regions = _geometry3d->region_count();
        if(num_regions > 1) {
            throw SimulationError(
                "FIXME: Real 3D not yet supported, but found {} regions.", num_regions);
        }
        agent.regionId = 0;
        anchor = _geometry3d->locate_in_region(agent.regionId, {agent.pos.x, agent.pos.y});
        if(anchor.face == SurfaceMesh::null_face()) {
            throw SimulationError("Agent {} not inside walkable area", agent.pos);
        }
    } else if(!_geometry->InsideGeometry(agent.pos)) {
        throw SimulationError("Agent {} not inside walkable area", agent.pos);
    }
    if(_journeys.count(agent.journeyId) == 0) {
        throw SimulationError("Unknown journey id: {}", agent.journeyId);
    }

    if(!_journeys.at(agent.journeyId)->ContainsStage(agent.stageId)) {
        throw SimulationError("Unknown stage id: {}", agent.stageId);
    }

    if(const auto agentModelType = ModelTypeOf(agent.model);
       agentModelType != _operationalDecisionSystem.ModelType()) {
        throw SimulationError(
            "Agent model data of type '{}' does not match the simulation's operational model "
            "'{}'",
            ToString(agentModelType),
            ToString(_operationalDecisionSystem.ModelType()));
    }

    if(_gatherer3d) {
        _operationalDecisionSystem.ValidateAgentOnSurface(agent, anchor.point, *_gatherer3d);
    } else {
        _operationalDecisionSystem.ValidateAgent(agent, _neighborhoodSearch, *_geometry);
    }

    _stageManager.HandleNewAgent(agent.stageId);
    _agents.emplace_back(std::move(agent));
    if(_gatherer3d) {
        _gatherer3d->add(_agents, anchor.point);
    } else {
        _neighborhoodSearch.AddAgent(_agents.back());
    }

    auto v = IteratorPair(std::prev(std::end(_agents)), std::end(_agents));
    _stategicalDecisionSystem.Run(_journeys, v, _stageManager);
    _tacticalDecisionSystem.Run(*_routingEngine, v, _gatherer3d ? _geometry3d.get() : nullptr);
    return _agents.back().id.getID();
}

void Simulation::MarkAgentForRemoval(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter = std::find_if(
        std::begin(_agents), std::end(_agents), [id](auto& agent) { return agent.id == id; });
    if(iter == std::end(_agents)) {
        throw SimulationError("Unknown agent id {}", id);
    }

    _removedAgentsInLastIteration.push_back(id);
}

const GenericAgent& Simulation::Agent(GenericAgent::ID id) const
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(_agents.begin(), _agents.end(), [id](auto& ped) { return id == ped.id; });
    if(iter == _agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

GenericAgent& Simulation::Agent(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(_agents.begin(), _agents.end(), [id](auto& ped) { return id == ped.id; });
    if(iter == _agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

const std::vector<GenericAgent::ID>& Simulation::RemovedAgents() const
{
    return _removedAgentsInLastIteration;
}

double Simulation::ElapsedTime() const
{
    return _clock.ElapsedTime();
}

double Simulation::DT() const
{
    return _clock.dT();
}

uint64_t Simulation::Iteration() const
{
    return _clock.Iteration();
}

size_t Simulation::AgentCount() const
{
    return _agents.size();
}

AgentContainer<GenericAgent>& Simulation::Agents()
{
    return _agents;
};

void Simulation::SwitchAgentJourney(
    GenericAgent::ID agent_id,
    Journey::ID journey_id,
    BaseStage::ID stage_id)
{
    JPS_TRACE_FUNC;
    const auto find_iter = _journeys.find(journey_id);
    if(find_iter == std::end(_journeys)) {
        throw SimulationError("Unknown Journey id {}", journey_id);
    }
    auto& journey = find_iter->second;
    if(!journey->ContainsStage(stage_id)) {
        throw SimulationError("Stage {} not part of Journey {}", stage_id, journey_id);
    }
    auto& agent = Agent(agent_id);
    agent.journeyId = journey_id;
    _stageManager.MigrateAgent(agent.stageId, stage_id);
    agent.stageId = stage_id;
}

std::vector<GenericAgent::ID> Simulation::AgentsInRange(Point p, double distance)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agents in Range", Debug);
    if(_gatherer3d) {
        // Horizontal-only interim: the vertical semantic of the public
        // queries (which level?) is an open API decision (Road-to-full-3D,
        // C8); under the single-region guard the exact 2D scan is correct.
        std::vector<GenericAgent::ID> result{};
        const auto radiusSquared = distance * distance;
        for(const auto& agent : _agents) {
            if(DistanceSquared(agent.pos, p) <= radiusSquared) {
                result.push_back(agent.id);
            }
        }
        return result;
    }
    const auto neighbors = _neighborhoodSearch.GetNeighboringAgents(p, distance);

    std::vector<GenericAgent::ID> neighborIds{};
    neighborIds.reserve(neighbors.size());
    std::transform(
        std::begin(neighbors),
        std::end(neighbors),
        std::back_inserter(neighborIds),
        [](const auto& agent) { return agent.id; });
    return neighborIds;
}

std::vector<GenericAgent::ID> Simulation::AgentsInPolygon(const std::vector<Point>& polygon)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agents in Polygon", Debug);
    const Polygon poly{polygon};
    if(!poly.IsConvex()) {
        throw SimulationError("Polygon needs to be simple and convex");
    }
    if(_gatherer3d) {
        // Same horizontal-only interim as AgentsInRange (the circle prefilter
        // of the 2D path is pure acceleration).
        std::vector<GenericAgent::ID> result{};
        for(const auto& agent : _agents) {
            if(poly.IsInside(agent.pos)) {
                result.push_back(agent.id);
            }
        }
        return result;
    }
    const auto [p, dist] = poly.ContainingCircle();

    const auto candidates = _neighborhoodSearch.GetNeighboringAgents(p, dist);
    std::vector<GenericAgent::ID> result{};
    result.reserve(candidates.size());
    std::for_each(
        std::begin(candidates), std::end(candidates), [&result, &poly](const auto& agent) {
            if(poly.IsInside(agent.pos)) {
                result.push_back(agent.id);
            }
        });
    return result;
}

OperationalModelType Simulation::ModelType() const
{
    return _operationalDecisionSystem.ModelType();
}

StageProxy Simulation::Stage(BaseStage::ID stageId)
{
    return _stageManager.Stage(stageId)->Proxy(this);
}
CollisionGeometry Simulation::Geo() const
{
    return *_geometry;
}

void Simulation::PushTimer(const std::string_view name, size_t probe_log_level)
{
    _timer.pushTimerProbe(name, probe_log_level);
}

void Simulation::PopTimer(const std::string_view name)
{
    _timer.popTimerProbe(name);
}

TimerEntry::duration_type Simulation::GetTimerDuration(const std::string_view name) const
{
    return _timer.getDuration(name);
}

std::map<std::string, TimerEntry::duration_type> Simulation::GetTimerDurations() const
{
    return _timer.getDurations();
}
