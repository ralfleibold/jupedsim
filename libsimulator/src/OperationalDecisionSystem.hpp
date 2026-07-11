// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "InformationForUpdate.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "SurfaceStep.hpp"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

class OperationalDecisionSystem
{
    std::unique_ptr<OperationalModel> _model{};

public:
    OperationalDecisionSystem(std::unique_ptr<OperationalModel>&& model) : _model(std::move(model))
    {
    }
    ~OperationalDecisionSystem() = default;
    OperationalDecisionSystem(const OperationalDecisionSystem& other) = delete;
    OperationalDecisionSystem& operator=(const OperationalDecisionSystem& other) = delete;
    OperationalDecisionSystem(OperationalDecisionSystem&& other) = delete;
    OperationalDecisionSystem& operator=(OperationalDecisionSystem&& other) = delete;

    OperationalModelType ModelType() const { return _model->Type(); }

    void
    Run(double dT,
        double /*t_in_sec*/,
        const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
        const CollisionGeometry& geometry,
        AgentContainer<GenericAgent>& agents) const
    {
        const auto agentCount = agents.size();

        // Phase 1: gather each agent's requested information. Batching the
        // world queries ahead of the model calls keeps the models pure
        // functions of (dT, agent, info).
        std::vector<InformationForUpdate> infos{};
        infos.reserve(agentCount);
        for(const auto& agent : agents) {
            infos.push_back(
                Gather(agent, _model->Requirements(agent), neighborhoodSearch, geometry));
        }

        // Phase 2: compute all updates from the frozen pre-step state.
        std::vector<OperationalModelUpdate> updates{};
        updates.reserve(agentCount);
        for(std::size_t i = 0; i < agentCount; ++i) {
            updates.push_back(_model->ComputeNewPosition(dT, agents[i], infos[i]));
        }

        // Phase 3: apply.
        for(std::size_t i = 0; i < agentCount; ++i) {
            _model->ApplyUpdate(updates[i], agents[i]);
        }
    }

    /// The 3D counterpart of Run: gather through the surface neighborhood
    /// (InformationGatherer3D), compute with the unchanged models, apply and
    /// re-anchor onto the surface. Transitional; replaces Run at the switch.
    void RunOnSurface(
        double dT,
        InformationGatherer3D& gatherer,
        const Geometry3D& geometry,
        AgentContainer<GenericAgent>& agents) const
    {
        run_surface_step(dT, *_model, gatherer, geometry, agents);
    }

    void ValidateAgent(
        const GenericAgent& agent,
        const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
        const CollisionGeometry& geometry) const
    {
        _model->CheckModelConstraint(
            agent,
            Gather(agent, _model->ConstraintRequirements(agent), neighborhoodSearch, geometry));
    }

    /// Validation counterpart of RunOnSurface: the candidate agent (anchored
    /// on the surface at @p anchor) is checked against the gatherer's current
    /// index, which it is not part of yet.
    void ValidateAgentOnSurface(
        const GenericAgent& agent,
        const Point3D& anchor,
        const InformationGatherer3D& gatherer) const
    {
        std::vector<LineSegment> wallBuffer{};
        _model->CheckModelConstraint(
            agent, gatherer.gather_at(anchor, _model->ConstraintRequirements(agent), wallBuffer));
    }

private:
    InformationForUpdate Gather(
        const GenericAgent& agent,
        const InformationRequirements& requirements,
        const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
        const CollisionGeometry& geometry) const
    {
        InformationForUpdate info{};
        if(requirements.neighborRadius) {
            info.neighbors =
                neighborhoodSearch.GetNeighboringAgents(agent.pos, *requirements.neighborRadius);
        }
        if(requirements.wallRadius) {
            // The approximate grid delivers all wall segments within its build
            // radius (4 m) of the agent -- over-inclusive, and silently capped
            // for larger requests, matching the previous direct-query behavior.
            const auto& walls = geometry.LineSegmentsInApproxDistanceTo(agent.pos);
            info.walls = std::span<const LineSegment>(walls.data(), walls.size());
        }
        return info;
    }
};
