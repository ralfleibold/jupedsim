// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "Clonable.hpp"
#include "Mesh.hpp"
#include "Point.hpp"
#include "RoutingEngine3D.hpp"

#include <cstddef>
#include <memory>
#include <variant>
#include <vector>

/// The legacy 2D routing engine (A* + funnel with 0.2 m wall clearance on the
/// CDT). Implements the RoutingEngine3D interface by ignoring z (queries) and
/// lifting results to z=0, so the simulation loop already speaks the 3D
/// interface before the switch.
class RoutingEngine : public RoutingEngine3D, public Clonable<RoutingEngine>
{
    CDT cdt{};
    std::unique_ptr<Mesh> mesh{};

public:
    RoutingEngine();
    explicit RoutingEngine(const PolyWithHoles& poly);
    ~RoutingEngine() override = default;

    // -- RoutingEngine3D interface (z ignored / lifted to z=0) --------------
    bool IsValidLocation(const Location& loc) const override;
    std::tuple<std::vector<Point3D>, double>
    GetShortestPath(const Point3D& source, const Location& target) override;
    Point GetNextWaypoint(const Point3D& source, const Location& target) override;
    Point GetOrientation(const Point3D& source, const Location& target) override;

    std::unique_ptr<RoutingEngine> Clone() const override;
    Point ComputeWaypoint(Point currentPosition, Point destination);
    std::vector<Point> ComputeAllWaypoints(Point currentPosition, Point destination);
    bool IsRoutable(Point p) const;
    void Update();

    const Mesh* MeshData() const { return mesh.get(); };

private:
    CDT::Face_handle find_face(K::Point_2) const;
    std::vector<Point>
    straightenPath(Point from, Point to, const std::vector<CDT::Face_handle>& path);
};
