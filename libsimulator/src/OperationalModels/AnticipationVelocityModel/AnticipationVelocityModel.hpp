// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"

#include <cstdint>
#include <random>
#include <span>

struct GenericAgent;

class AnticipationVelocityModel : public OperationalModel
{
    double _cutOffRadius{3};
    /// Add a small outward component to maintain minimum distance from walls.
    double _pushoutStrength;
    mutable std::mt19937 gen;

public:
    AnticipationVelocityModel(double pushoutStrength, uint64_t rng_seed);
    ~AnticipationVelocityModel() override = default;
    OperationalModelType Type() const override;
    InformationRequirements Requirements(const GenericAgent& agent) const override;
    InformationRequirements ConstraintRequirements(const GenericAgent& agent) const override;
    OperationalModelUpdate ComputeNewPosition(
        double dT,
        const GenericAgent& ped,
        const InformationForUpdate& info) const override;
    void ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const override;
    void CheckModelConstraint(const GenericAgent& agent, const InformationForUpdate& info)
        const override;

private:
    double OptimalSpeed(const GenericAgent& ped, double spacing, double time_gap) const;
    Point CalculateInfluenceDirection(
        const Point& desiredDirection,
        const Point& predictedDirection) const;
    double
    GetSpacing(const GenericAgent& ped1, const GenericAgent& ped2, const Point& direction) const;
    Point NeighborRepulsion(const GenericAgent& ped1, const GenericAgent& ped2) const;

    Point HandleWallAvoidance(
        const Point& direction,
        const Point& agentPosition,
        double agentRadius,
        std::span<const LineSegment> boundary,
        double wallBufferDistance) const;

    Point
    UpdateDirection(const GenericAgent& ped, const Point& calculatedDirection, double dt) const;
};
