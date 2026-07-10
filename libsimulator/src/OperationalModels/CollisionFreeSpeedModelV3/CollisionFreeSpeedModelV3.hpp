// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"

struct GenericAgent;

class CollisionFreeSpeedModelV3 : public OperationalModel
{
    double _cutOffRadius{3};

public:
    CollisionFreeSpeedModelV3() = default;
    ~CollisionFreeSpeedModelV3() override = default;
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
    double
    GetSpacing(const GenericAgent& ped1, const GenericAgent& ped2, const Point& direction) const;
    Point BoundaryRepulsion(const GenericAgent& ped, const LineSegment& boundary_segment) const;
};
