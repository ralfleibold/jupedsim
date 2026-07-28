// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GenericAgent.hpp"
#include "GeometricFunctions.hpp"
#include "Geometry/Geometry2D.hpp"
#include "LineSegment.hpp"
#include "NeighborhoodSearch.hpp"
#include "Point.hpp"

#include <algorithm>
#include <concepts>
#include <iterator>
#include <ranges>
#include <vector>

using OperationalModelState = GenericAgent::ModelState;

class EnvironmentQuery
{
    const Geometry2D& _geometry;
    const NeighborhoodSearch<GenericAgent>& _nsearch;

public:
    EnvironmentQuery(const Geometry2D& geometry, const NeighborhoodSearch<GenericAgent>& nsearch)
        : _geometry(geometry), _nsearch(nsearch)
    {
    }

    struct AcceptAll {
        bool operator()(const Point&) const { return true; }
    };

    // Returns all agents within 'radius' of 'agent', excluding 'agent' itself.
    // An optional predicate 'filter' further filters the result; it is never called
    // with 'agent'. Example:
    //   query.OtherAgentsInRange(agent, r, [&envQuery, from = agent.position](const Point& to) {
    //   return envQuery.NoGeometryBetween(from, to);})
    template <std::predicate<const Point&> Pred = AcceptAll>
    std::vector<GenericAgent>
    OtherAgentsInRange(const GenericAgent& agent, double radius, Pred filter = {}) const
    {
        auto neighbors = _nsearch.GetNeighboringAgents(agent.position, radius);
        std::erase_if(neighbors, [&](const GenericAgent& candidate) {
            return candidate.id == agent.id || !filter(candidate.position);
        });
        return neighbors;
    }

    template <std::predicate<const Point&> Pred = AcceptAll>
    std::vector<GenericAgent>
    AgentsInRange(const Point& from, double radius, Pred filter = {}) const
    {
        auto neighbors = _nsearch.GetNeighboringAgents(from, radius);
        std::erase_if(
            neighbors, [&](const GenericAgent& candidate) { return !filter(candidate.position); });
        return neighbors;
    }

    bool NoGeometryBetween(const Point& from, const Point& to) const
    {
        const LineSegment los{from, to};
        const double dist = Distance(from, to);
        auto blocked = [&los](const auto& boundaries) {
            return std::any_of(boundaries.begin(), boundaries.end(), [&los](const auto& seg) {
                return intersects(los, seg);
            });
        };
        return !blocked(LineSegmentsInRange(from, dist));
    }

    Geometry2D::LineSegmentRange LineSegmentsInRange(const Point& p, double distance = -1.0) const
    {
        if(distance < 0.0) {
            return _geometry.LineSegmentsInApproxDistanceTo(p);
        } else {
            return _geometry.LineSegmentsInDistanceTo(distance, p);
        }
    }

    bool InsideGeometry(const Point& p) const { return _geometry.InsideGeometry(p); }

    const Geometry2D& Geometry() const { return _geometry; }
};
