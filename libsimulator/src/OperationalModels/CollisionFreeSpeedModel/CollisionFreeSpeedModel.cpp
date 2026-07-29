// SPDX-License-Identifier: LGPL-3.0-or-later
#include "CollisionFreeSpeedModel.hpp"

#include "AgentView.hpp"
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
#include <vector>

CollisionFreeSpeedModel::CollisionFreeSpeedModel(
    double strengthNeighborRepulsion_,
    double rangeNeighborRepulsion_,
    double strengthGeometryRepulsion_,
    double rangeGeometryRepulsion_)
    : strengthNeighborRepulsion(strengthNeighborRepulsion_)
    , rangeNeighborRepulsion(rangeNeighborRepulsion_)
    , strengthGeometryRepulsion(strengthGeometryRepulsion_)
    , rangeGeometryRepulsion(rangeGeometryRepulsion_)
{
}

OperationalModelType CollisionFreeSpeedModel::Type() const
{
    return OperationalModelType::COLLISION_FREE_SPEED;
}

Point CollisionFreeSpeedModel::ComputeNextState(
    const OperationalModelState& current,
    OperationalModelState& next,
    const AgentStep& step) const
{
    const auto& model = std::get<State>(current);
    const auto neighborhood =
        step.other_agents_in_range(_cutOffRadius, [&step](const NeighborView& n) {
            return step.no_geometry_between(n.relative_position);
        });

    Point neighborRepulsion{};
    for(const auto& neighbor : neighborhood) {
        neighborRepulsion += NeighborRepulsion(model, neighbor);
    }

    Point boundaryRepulsion{};
    for(const auto& wall : step.walls_nearby()) {
        boundaryRepulsion += BoundaryRepulsion(model, wall);
    }

    const auto desired_direction = step.to_next_target().Normalized();
    auto direction = (desired_direction + neighborRepulsion + boundaryRepulsion).Normalized();
    if(direction == Point{}) {
        direction = model.orientation;
    }
    auto spacing = std::numeric_limits<double>::max();
    for(const auto& neighbor : neighborhood) {
        spacing = std::min(spacing, GetSpacing(model, neighbor, direction));
    }

    const auto optimal_speed = OptimalSpeed(model, spacing, model.timeGap);
    const auto velocity = direction * optimal_speed;
    std::get<State>(next).orientation = direction;
    return velocity * step.dt();
}

void CollisionFreeSpeedModel::CheckModelConstraint(const GenericAgent& agent, const AgentView& view)
    const
{
    const auto& model = std::get<State>(agent.model);

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

    const auto neighbors = view.other_agents_in_range(2.0);
    for(const auto& neighbor : neighbors) {
        const auto& neighbor_model = std::get<State>(*neighbor.state);
        const auto contanctdDist = r + neighbor_model.radius;
        const auto distance = neighbor.relative_position.Norm();
        if(contanctdDist >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}",
                agent.position,
                agent.position + neighbor.relative_position,
                distance);
        }
    }

    if(!view.walls_in_range(r).empty()) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance "
            "<= {}",
            agent.position,
            r);
    }
}

double
CollisionFreeSpeedModel::OptimalSpeed(const State& self, double spacing, double time_gap) const
{
    return std::min(std::max(spacing / time_gap, 0.0), self.v0);
}

double CollisionFreeSpeedModel::GetSpacing(
    const State& self,
    const NeighborView& neighbor,
    const Point& direction) const
{
    const auto& other = std::get<State>(*neighbor.state);
    const auto distp12 = neighbor.relative_position;
    const auto inFront = direction.ScalarProduct(distp12) >= 0;
    if(!inFront) {
        return std::numeric_limits<double>::max();
    }

    const auto left = direction.Rotate90Deg();
    const auto l = self.radius + other.radius;
    bool inCorridor = std::abs(left.ScalarProduct(distp12)) <= l;
    if(!inCorridor) {
        return std::numeric_limits<double>::max();
    }
    return distp12.Norm() - l;
}
Point CollisionFreeSpeedModel::NeighborRepulsion(const State& self, const NeighborView& neighbor)
    const
{
    const auto& other = std::get<State>(*neighbor.state);
    const auto [distance, direction] = neighbor.relative_position.NormAndNormalized();
    const auto l = self.radius + other.radius;
    return direction *
           -(this->strengthNeighborRepulsion * exp((l - distance) / this->rangeNeighborRepulsion));
}

Point CollisionFreeSpeedModel::BoundaryRepulsion(
    const State& self,
    const LineSegment& boundary_segment) const
{
    const auto dist_vec = boundary_segment.ShortestPoint(Point{});
    const auto [dist, e_iw] = dist_vec.NormAndNormalized();
    const auto l = self.radius;
    const auto R_iw =
        -this->strengthGeometryRepulsion * exp((l - dist) / this->rangeGeometryRepulsion);
    return e_iw * R_iw;
}
