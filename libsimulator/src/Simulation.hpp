// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "AgentRemovalSystem.hpp"
#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "InformationGatherer3D.hpp"
#include "Journey.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalDecisionSystem.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "RoutingEngine3D.hpp"
#include "SimulationClock.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "StageManager.hpp"
#include "StageSystem.hpp"
#include "StrategicalDesicionSystem.hpp"
#include "TacticalDecisionSystem.hpp"
#include "Timing.hpp"
#include "Tracing.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

class Simulation
{
    SimulationClock _clock;
    StrategicalDecisionSystem _stategicalDecisionSystem{};
    TacticalDecisionSystem _tacticalDecisionSystem{};
    OperationalDecisionSystem _operationalDecisionSystem;
    AgentRemovalSystem<GenericAgent> _agentRemovalSystem{};
    StageManager _stageManager{};
    StageSystem _stageSystem{};
    NeighborhoodSearch<GenericAgent> _neighborhoodSearch{2.2};
    std::unique_ptr<Geometry3D> _geometry3d{};
    /// The projected 2D view owned by _geometry3d. Required (non-null) only
    /// on the 2D operational path; mesh-built geometries in surface mode
    /// have none.
    const CollisionGeometry* _geometry{};
    std::unique_ptr<RoutingEngine3D> _routingEngine{};
    /// Present iff the operational step runs on the surface (run_in_2d ==
    /// false): gather + apply go through the 3D machinery, agents carry their
    /// region anchor. Transitional A/B seam for the 2D/3D parity gate.
    std::unique_ptr<InformationGatherer3D> _gatherer3d{};
    AgentContainer<GenericAgent> _agents;
    std::vector<GenericAgent::ID> _removedAgentsInLastIteration;
    std::unordered_map<Journey::ID, std::unique_ptr<Journey>> _journeys;
    Timer _timer{};
    enum LogLevel { General = 1, Detailed = 2, Debug = 3 };

public:
    /// Simulation owns @p geometry (the single source of truth); the injected
    /// @p routingEngine must have been constructed against that same geometry
    /// (it borrows, never owns).
    /// @p runIn2d selects the operational path: true (default) = the legacy
    /// 2D gather/apply, false = gather + apply + re-anchor on the surface
    /// mesh. On flat geometry both must produce identical trajectories --
    /// this is the transitional parity switch, removed once 3D is the default.
    /// The 2D path additionally requires the projected 2D view
    /// (collision_geometry() != nullptr, i.e. built from a polygon) and a
    /// single-region geometry; the surface path takes mesh-built,
    /// multi-region geometries.
    Simulation(
        std::unique_ptr<OperationalModel>&& operationalModel,
        std::unique_ptr<Geometry3D>&& geometry,
        std::unique_ptr<RoutingEngine3D>&& routingEngine,
        double dT,
        bool runIn2d = true);
    Simulation(const Simulation& other) = delete;
    Simulation& operator=(const Simulation& other) = delete;
    Simulation(Simulation&& other) = delete;
    Simulation& operator=(Simulation&& other) = delete;
    ~Simulation() = default;
    const SimulationClock& Clock() const;
    void SetTracing(bool on);
    void Iterate();
    Journey::ID AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages);
    /// @p zHint disambiguates vertically stacked sheets in surface mode:
    /// the stage's representative point (waypoint position, exit centroid,
    /// slots) anchors on the sheet nearest the hint (within ZHintTolerance).
    /// Ignored on the 2D path.
    BaseStage::ID AddStage(const StageDescription stageDescription, double zHint = 0.0);
    void MarkAgentForRemoval(GenericAgent::ID id);
    const std::vector<GenericAgent::ID>& RemovedAgents() const;
    size_t AgentCount() const;
    double ElapsedTime() const;
    double DT() const;
    void
    SwitchAgentJourney(GenericAgent::ID agent_id, Journey::ID journey_id, BaseStage::ID stage_id);
    uint64_t Iteration() const;
    std::vector<GenericAgent::ID> AgentsInRange(Point p, double distance);
    /// Returns IDs of all agents inside the defined polygon
    /// @param polygon Required to be a simple convex polygon with CCW ordering.
    std::vector<GenericAgent::ID> AgentsInPolygon(const std::vector<Point>& polygon);
    /// @p zHint disambiguates vertically stacked sheets in surface mode: the
    /// agent anchors on the sheet whose surface z at agent.pos is nearest the
    /// hint and within ZHintTolerance. Ignored on the 2D path.
    GenericAgent::ID AddAgent(GenericAgent agent, double zHint = 0.0);
    const GenericAgent& Agent(GenericAgent::ID id) const;
    GenericAgent& Agent(GenericAgent::ID id);
    AgentContainer<GenericAgent>& Agents();
    OperationalModelType ModelType() const;
    StageProxy Stage(BaseStage::ID stageId);
    CollisionGeometry Geo() const;
    void PushTimer(const std::string_view name, size_t probe_log_level = 0);
    void PopTimer(const std::string_view name);
    void SetTimerLogLevel(int level) { _timer.setLogLevel(level); };
    TimerEntry::duration_type GetTimerDuration(const std::string_view name) const;
    std::map<std::string, TimerEntry::duration_type> GetTimerDurations() const;
};
