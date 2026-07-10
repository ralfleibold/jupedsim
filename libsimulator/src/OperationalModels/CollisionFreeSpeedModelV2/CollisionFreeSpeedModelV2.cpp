// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModelV2.hpp"

#include "CollisionFreeSpeedModelV2Data.hpp"
#include "CollisionFreeSpeedModelV2Update.hpp"
#include "GenericAgent.hpp"
#include "GeometricFunctions.hpp"
#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationError.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>

OperationalModelType CollisionFreeSpeedModelV2::Type() const
{
    return OperationalModelType::COLLISION_FREE_SPEED_V2;
}

InformationRequirements CollisionFreeSpeedModelV2::Requirements(const GenericAgent&) const
{
    return {.neighborRadius = _cutOffRadius, .wallRadius = _cutOffRadius};
}

InformationRequirements
CollisionFreeSpeedModelV2::ConstraintRequirements(const GenericAgent& agent) const
{
    const auto& model = std::get<CollisionFreeSpeedModelV2Data>(agent.model);
    return {.neighborRadius = 2., .wallRadius = model.radius};
}

OperationalModelUpdate CollisionFreeSpeedModelV2::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const InformationForUpdate& info) const
{
    auto neighborhood = info.neighbors;
    const auto& boundary = info.walls;

    // Remove any agent from the neighborhood that is obstructed by geometry and the current
    // agent
    neighborhood.erase(
        std::remove_if(
            std::begin(neighborhood),
            std::end(neighborhood),
            [&ped, &boundary](const auto& neighbor) {
                if(ped.id == neighbor.id) {
                    return true;
                }
                const auto agent_to_neighbor = LineSegment(ped.pos, neighbor.pos);
                if(std::find_if(
                       boundary.begin(),
                       boundary.end(),
                       [&agent_to_neighbor](const auto& boundary_segment) {
                           return intersects(agent_to_neighbor, boundary_segment);
                       }) != boundary.end()) {
                    return true;
                }

                return false;
            }),
        std::end(neighborhood));

    const auto neighborRepulsion = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        Point{},
        [&ped, this](const auto& res, const auto& neighbor) {
            return res + NeighborRepulsion(ped, neighbor);
        });

    const auto boundaryRepulsion = std::accumulate(
        boundary.begin(),
        boundary.end(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + BoundaryRepulsion(ped, element);
        });

    const auto desired_direction = (ped.destination - ped.pos).Normalized();
    auto direction = (desired_direction + neighborRepulsion + boundaryRepulsion).Normalized();
    const auto& model = std::get<CollisionFreeSpeedModelV2Data>(ped.model);
    if(direction == Point{}) {
        direction = model.orientation;
    }
    const auto spacing = std::accumulate(
        std::begin(neighborhood),
        std::end(neighborhood),
        std::numeric_limits<double>::max(),
        [&ped, &direction, this](const auto& res, const auto& neighbor) {
            return std::min(res, GetSpacing(ped, neighbor, direction));
        });

    const auto optimal_speed = OptimalSpeed(ped, spacing, model.timeGap);
    const auto velocity = direction * optimal_speed;
    return CollisionFreeSpeedModelV2Update{ped.pos + velocity * dT, direction};
};

void CollisionFreeSpeedModelV2::ApplyUpdate(const OperationalModelUpdate& upd, GenericAgent& agent)
    const
{
    const auto& update = std::get<CollisionFreeSpeedModelV2Update>(upd);
    auto& model = std::get<CollisionFreeSpeedModelV2Data>(agent.model);
    agent.pos = update.position;
    model.orientation = update.orientation;
}

void CollisionFreeSpeedModelV2::CheckModelConstraint(
    const GenericAgent& agent,
    const InformationForUpdate& info) const
{
    const auto& model = std::get<CollisionFreeSpeedModelV2Data>(agent.model);

    const auto r = model.radius;
    constexpr double rMin = 0.;
    constexpr double rMax = 2.;
    validateConstraint(r, rMin, rMax, "radius", true);

    const auto v0 = model.v0;
    constexpr double v0Min = 0.;
    constexpr double v0Max = 10.;
    validateConstraint(v0, v0Min, v0Max, "v0");

    const auto timeGap = model.timeGap;
    constexpr double timeGapMin = 0.1;
    constexpr double timeGapMax = 10.;
    validateConstraint(timeGap, timeGapMin, timeGapMax, "timeGap");

    for(const auto& neighbor : info.neighbors) {
        if(agent.id == neighbor.id) {
            continue;
        }
        const auto& neighbor_model = std::get<CollisionFreeSpeedModelV2Data>(neighbor.model);
        const auto contanctdDist = r + neighbor_model.radius;
        const auto distance = (agent.pos - neighbor.pos).Norm();
        if(contanctdDist >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}",
                agent.pos,
                neighbor.pos,
                distance);
        }
    }

    // info.walls may be over-inclusive, apply the exact distance here.
    const auto tooClose =
        std::any_of(info.walls.begin(), info.walls.end(), [&agent, r](const auto& segment) {
            return segment.DistTo(agent.pos) <= r;
        });
    if(tooClose) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance "
            "<= {}",
            agent.pos,
            r);
    }
}

double CollisionFreeSpeedModelV2::OptimalSpeed(
    const GenericAgent& ped,
    double spacing,
    double time_gap) const
{
    const auto& model = std::get<CollisionFreeSpeedModelV2Data>(ped.model);
    return std::min(std::max(spacing / time_gap, 0.0), model.v0);
}

double CollisionFreeSpeedModelV2::GetSpacing(
    const GenericAgent& ped1,
    const GenericAgent& ped2,
    const Point& direction) const
{
    const auto& model1 = std::get<CollisionFreeSpeedModelV2Data>(ped1.model);
    const auto& model2 = std::get<CollisionFreeSpeedModelV2Data>(ped2.model);
    const auto distp12 = ped2.pos - ped1.pos;
    const auto inFront = direction.ScalarProduct(distp12) >= 0;
    if(!inFront) {
        return std::numeric_limits<double>::max();
    }

    const auto left = direction.Rotate90Deg();
    const auto l = model1.radius + model2.radius;
    bool inCorridor = std::abs(left.ScalarProduct(distp12)) <= l;
    if(!inCorridor) {
        return std::numeric_limits<double>::max();
    }
    return distp12.Norm() - l;
}
Point CollisionFreeSpeedModelV2::NeighborRepulsion(
    const GenericAgent& ped1,
    const GenericAgent& ped2) const
{
    const auto distp12 = ped2.pos - ped1.pos;
    const auto [distance, direction] = distp12.NormAndNormalized();
    const auto& model1 = std::get<CollisionFreeSpeedModelV2Data>(ped1.model);
    const auto& model2 = std::get<CollisionFreeSpeedModelV2Data>(ped2.model);
    const auto l = model1.radius + model2.radius;
    return direction * -(model1.strengthNeighborRepulsion *
                         exp((l - distance) / model1.rangeNeighborRepulsion));
}

Point CollisionFreeSpeedModelV2::BoundaryRepulsion(
    const GenericAgent& ped,
    const LineSegment& boundary_segment) const
{
    const auto pt = boundary_segment.ShortestPoint(ped.pos);
    const auto dist_vec = pt - ped.pos;
    const auto [dist, e_iw] = dist_vec.NormAndNormalized();
    const auto& model = std::get<CollisionFreeSpeedModelV2Data>(ped.model);
    const auto l = model.radius;
    const auto R_iw =
        -model.strengthGeometryRepulsion * exp((l - dist) / model.rangeGeometryRepulsion);
    return e_iw * R_iw;
}
