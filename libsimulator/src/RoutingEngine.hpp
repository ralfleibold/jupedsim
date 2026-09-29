// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Geometry/Geometry.hpp"
#include "Geometry/Location.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"

#include <cstddef>
#include <span>
#include <variant>
#include <vector>

/// A destination registered with `RoutingEngine::AddDestination`.
enum class DestinationId : std::size_t {};

/// One part of a destination: a polygon within a single region.
struct DestinationArea {
    std::size_t region;
    Polygon polygon;
};

using RoutingTarget = std::variant<DestinationId, Location>;

class RoutingEngine
{
public:
    /// Borrows @p geometry (non-owning); the caller keeps it alive for the engine's lifetime.
    explicit RoutingEngine(const Geometry& geometry);
    ~RoutingEngine() = default;

    RoutingEngine(const RoutingEngine&) = delete;
    RoutingEngine& operator=(const RoutingEngine&) = delete;
    RoutingEngine(RoutingEngine&&) = delete;
    RoutingEngine& operator=(RoutingEngine&&) = delete;

    /// Registers @p areas as one destination. Routing will target the one that has the least
    /// expensive cost towards it.
    DestinationId AddDestination(std::span<const DestinationArea> areas);

    /// Unit vector from @p from along the route to @p to, projected to x/y. Zero once @p from
    /// has reached @p to.
    Point GetOrientation(const Location& from, const RoutingTarget& to);

private:
    /// The part of @p destination the way from @p from is shortest to.
    const Location& nearest(const Location& from, const std::vector<Location>& destination);

    const Geometry& _geometry;
    SurfaceMeshShortestPathRoutingEngine _shortestPath;
    /// Per destination: the centroid of each of its areas.
    std::vector<std::vector<Location>> _destinations{};
};
