// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Geometry3D.hpp"
#include "RoutingEngine3D.hpp"

#include <CGAL/Surface_mesh_shortest_path.h>

#include <memory>
#include <tuple>
#include <vector>

class SurfaceMeshShortestPathRoutingEngine : public RoutingEngine3D
{
public:
    explicit SurfaceMeshShortestPathRoutingEngine(std::unique_ptr<Geometry3D> geometry);
    ~SurfaceMeshShortestPathRoutingEngine() override = default;

    bool IsValidLocation(const Location& loc) const override;

    std::tuple<std::vector<Point3D>, double>
    GetShortestPath(const Point3D& source, const Location& target) override;

    Point GetOrientation(const Point3D& source, const Location& target) override;

private:
    std::unique_ptr<Geometry3D> _geometry;
};
