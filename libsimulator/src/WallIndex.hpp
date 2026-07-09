// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"

#include <vector>

/// Wall queries on 3D surface.
class WallIndex
{
    /// A border edge. It might be only a segment of the original mesh edge as
    /// there is some "headroom" check.
    struct Wall {
        Segment3D segment;
        /// Vertical headroom above `segment`: the height of the nearest walkable
        /// sheet directly above it, minus the segment's own height (+infinity if
        /// nothing is above).
        double headroom;
    };
    std::vector<Wall> _walls{};

public:
    explicit WallIndex(const SurfaceMesh& mesh);

    /// Number of walls (border edges, split at headroom boundaries).
    std::size_t wall_count() const { return _walls.size(); }

    /// "Wall" edges within horizontal distance `radius` of `p` whose height at
    /// the (2D-)closest footpoint differs compared to p.z by less than `height`.
    std::vector<Segment3D> get_near_walls(const Point3D& p, double radius, double height) const;

    /// True iff the 2D projection of a->b crosses no "wall" edge whose
    /// interpolated height at the crossing differs from the height of a->b at
    /// that point by less than `height`.
    bool is_visible(const Point3D& a, const Point3D& b, double height) const;

    /// ranges-filter (--> returns filter function):
    ///  `other` is visible from `p` (no blocking wall in between).
    auto is_visible_from(const Point3D& p, double height) const
    {
        return [this, p, height](const Point3D& other) { return is_visible(p, other, height); };
    }
};
