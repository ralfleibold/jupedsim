// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "Geometry3D.hpp"
#include "InformationGatherer3D.hpp"
#include "NeighborhoodSearch.hpp"
#include "SimulationError.hpp"
#include "Stage.hpp"
#include "StageManager.hpp"

#include <vector>

class StageSystem
{

public:
    StageSystem() {}
    ~StageSystem() = default;
    StageSystem(const StageSystem& other) = delete;
    StageSystem& operator=(const StageSystem& other) = delete;
    StageSystem(StageSystem&& other) = delete;
    StageSystem& operator=(StageSystem&& other) = delete;

    void
    Run(StageManager& stageManager,
        const NeighborhoodSearch<GenericAgent>& neighborhoodSearch,
        const CollisionGeometry& geometry)
    {
        update_stages(stageManager, stage_query_2d(neighborhoodSearch, geometry));
    }

    /// Surface counterpart of Run: slot surroundings come from the surface
    /// gatherer. Slots anchor on the sheet nearest the stage's z (the height
    /// Simulation::AddStage anchored the stage at).
    void RunOnSurface(
        StageManager& stageManager,
        const InformationGatherer3D& gatherer,
        const Geometry3D& geometry)
    {
        std::vector<LineSegment> wallBuffer{};
        update_stages(
            stageManager, [&gatherer, &geometry, &wallBuffer](Point p, double radius, double z) {
                const auto anchor = geometry.locate_near_z({p.x, p.y}, z, ZHintTolerance);
                if(anchor.face == SurfaceMesh::null_face()) {
                    throw SimulationError("Stage slot {} is not on the walkable surface", p);
                }
                return gatherer.gather_at(
                    anchor.point, {.neighborRadius = radius, .wallRadius = radius}, wallBuffer);
            });
    }

private:
    template <typename QueryAt>
    void update_stages(StageManager& stageManager, QueryAt&& queryAt)
    {
        for(auto& [_, stage] : stageManager.Stages()) {
            if(auto* updatable_stage = dynamic_cast<NotifiableWaitingSet*>(stage.get());
               updatable_stage != nullptr) {
                updatable_stage->Update(queryAt);
            } else if(auto* updatable_stage = dynamic_cast<NotifiableQueue*>(stage.get());
                      updatable_stage != nullptr) {
                updatable_stage->Update(queryAt);
            }
        }
    }
};
