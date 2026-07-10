// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "Point.hpp"

#include <tuple>
#include <vector>

using Location = Point3D; // [RL] TODO: Support more than just points

/// Pure interface for 3D routing engines.
class RoutingEngine3D
{
public:
    RoutingEngine3D() = default;
    virtual ~RoutingEngine3D() = default;

    // Non-copyable and non-movable
    RoutingEngine3D(const RoutingEngine3D&) = delete;
    RoutingEngine3D& operator=(const RoutingEngine3D&) = delete;
    RoutingEngine3D(RoutingEngine3D&&) = delete;
    RoutingEngine3D& operator=(RoutingEngine3D&&) = delete;

    /// Checks whether the provided location (3D-point or polygon)
    /// is on walkable surface taking wall clearance into account.
    /// @param loc location (Point or Polygon) to check
    /// @return true if the location projects onto the walkable surface
    virtual bool IsValidLocation(const Location& loc) const = 0;

    /// Compute the shortest path from @p source to @p target.
    /// @param source where to route from
    /// @param target where to route to
    /// @return tuple of (path, cost): the path includes source as first element
    ///         and the target as last element; cost is typically geodesic distance
    ///         along it, but not necessarily (e.g. floor fields with slowness field).
    virtual std::tuple<std::vector<Point3D>, double>
    GetShortestPath(const Point3D& source, const Location& target) = 0;

    /// Next waypoint of the shortest path from @p source to @p target,
    /// projected to x/y; @p source's x/y if already at the target.
    ///
    /// Transitional: the operational models consume `agent.destination` as a
    /// *point* (GCFM's J_EPS_GOAL goal freeze and WarpDriver's at-goal guard
    /// need the distance, not just a direction). Once routing moves to floor
    /// fields a "next waypoint" stops being meaningful and this collapses
    /// into GetOrientation plus an explicit at-goal signal.
    /// @param source where to route from
    /// @param target where to route to
    /// @return 2D next waypoint
    virtual Point GetNextWaypoint(const Point3D& source, const Location& target) = 0;

    /// Get orientation to next point of the shortest path from @p source to
    /// @p target, projected to x/y.
    /// @param source where to route from
    /// @param target where to route to
    /// @return 2D orientation to the next waypoint
    virtual Point GetOrientation(const Point3D& source, const Location& target) = 0;
};
