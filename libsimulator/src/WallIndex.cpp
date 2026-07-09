// SPDX-License-Identifier: LGPL-3.0-or-later
#include "WallIndex.hpp"

#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <variant>
#include <vector>

namespace
{
using Point2D = SurfaceKernel::Point_2;
using Segment2D = SurfaceKernel::Segment_2;

Point2D remove_z(const Point3D& p)
{
    return {p.x(), p.y()};
}

Segment2D flatten(const Segment3D& s)
{
    return {remove_z(s.source()), remove_z(s.target())};
}

/// Height of `seg` at the point of its 2D projection closest to `at`.
double height_at(const Segment3D& seg, const Point2D& at)
{
    const auto source = remove_z(seg.source());
    const auto dir = remove_z(seg.target()) - source;
    const auto len2 = dir.squared_length();
    // Just safety check for debug mode: We should not have any vertical edges via contract
    assert(len2 > 0 && "FATAL: vertical segment detected");
    // we want the closest point on the segment, not in general to an extended line.
    const auto t = std::clamp(((at - source) * dir) / len2, 0.0, 1.0);
    return seg.source().z() + t * (seg.target().z() - seg.source().z());
}

/// Linear interpolation of the segment between a and b wrt to value t. (t=0 --> return a)
Point3D lerp(const Point3D& a, const Point3D& b, double t)
{
    return {a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()), a.z() + t * (b.z() - a.z())};
}

/// Project `p` (assumed above the edge line) onto `edge`: parameter t in [0,1]
/// along the edge, and the vertical gap from the edge to `p` at that t.
std::pair<double, double> project_onto(const Segment3D& edge, const Point3D& p)
{
    const auto a = edge.source();
    const auto b = edge.target();
    const auto a2 = remove_z(a);
    const auto dir = remove_z(b) - a2;
    const auto len2 = dir.squared_length();
    assert(len2 > 0 && "FATAL: vertical edge in the wall index");
    const double t = std::clamp(((remove_z(p) - a2) * dir) / len2, 0.0, 1.0);
    return {t, p.z() - (a.z() + t * (b.z() - a.z()))};
}

/// The vertical "shadow" of one covering face on `edge`: the stretch [t0,t1] of the edge
/// (edge parameter t in [0,1]) over which that face lies above, with the vertical
/// gap to it running linearly from g0 at t0 to g1 at t1.
struct Cover {
    double t0, g0, t1, g1;
};

/// The covers over `edge`: one `Cover` (projected shadow) per mesh face lying
/// above it, found *exactly* by intersecting the vertical "curtain" above the
/// edge (base just above the edge, up to `zTop`) with the mesh.
std::vector<Cover> covers_above(const AABBTree& tree, const Segment3D& edge, double zTop)
{
    const auto a = edge.source();
    const auto b = edge.target();
    // Raise the base slightly so the edge's own incident face (gap 0) is not
    // mistaken for cover.
    const Point3D aLo{a.x(), a.y(), a.z() + 1e-4};
    const Point3D bLo{b.x(), b.y(), b.z() + 1e-4};
    const Point3D aHi{a.x(), a.y(), zTop};
    const Point3D bHi{b.x(), b.y(), zTop};
    std::vector<Cover> covers{};
    for(const auto& tri :
        {SurfaceKernel::Triangle_3{aLo, bLo, bHi}, SurfaceKernel::Triangle_3{aLo, bHi, aHi}}) {
        std::vector<AABBTree::Intersection_and_primitive_id<SurfaceKernel::Triangle_3>::Type>
            hits{};
        tree.all_intersections(tri, std::back_inserter(hits));
        for(const auto& [where, face] : hits) {
            const auto* seg = std::get_if<Segment3D>(&where);
            if(!seg) {
                continue; // a tangential point covers no interval
            }
            const auto [t0, g0] = project_onto(edge, seg->source());
            const auto [t1, g1] = project_onto(edge, seg->target());
            const Cover c = (t0 <= t1) ? Cover{t0, g0, t1, g1} : Cover{t1, g1, t0, g0};
            if(c.t1 - c.t0 > 1e-9) {
                covers.push_back(c);
            }
        }
    }
    return covers;
}
} // namespace

WallIndex::WallIndex(const SurfaceMesh& mesh)
{
    const AABBTree tree(mesh.faces().begin(), mesh.faces().end(), mesh);
    double zTop = -std::numeric_limits<double>::infinity();
    for(const auto v : mesh.vertices()) {
        zTop = std::max(zTop, mesh.point(v).z());
    }
    zTop += 1.0; // curtain reaches above the whole mesh

    for(const auto e : mesh.edges()) {
        if(!mesh.is_border(e)) {
            continue;
        }
        const Segment3D edge{mesh.point(mesh.vertex(e, 0)), mesh.point(mesh.vertex(e, 1))};
        const auto covers = covers_above(tree, edge, zTop);

        // Split the border at every coverage boundary; each piece carries the
        // most restrictive headroom over it (+infinity where nothing is above),
        // so a partly-covered wall keeps its uncovered stretches as real walls.
        // [RL, TODO] No coalescing: adjacent sub-segments with equal headroom
        // (and collinear neighbouring border edges) stay separate walls. Merging
        // them would shrink the linear scan in is_visible()/get_near_walls().
        std::vector<double> cuts{0.0, 1.0};
        for(const auto& c : covers) {
            cuts.push_back(c.t0);
            cuts.push_back(c.t1);
        }
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());

        for(std::size_t k = 0; k + 1 < cuts.size(); ++k) {
            const double ta = cuts[k];
            const double tb = cuts[k + 1];
            if(tb - ta < 1e-9) {
                continue;
            }
            const double mid = 0.5 * (ta + tb);
            double headroom = std::numeric_limits<double>::infinity();
            for(const auto& c : covers) {
                if(c.t0 <= mid && mid <= c.t1) {
                    const auto gap_at = [&c](double t) {
                        return c.g0 + (t - c.t0) / (c.t1 - c.t0) * (c.g1 - c.g0);
                    };
                    // Linear over the sub-interval -> the minimum is at an end.
                    headroom = std::min({headroom, gap_at(ta), gap_at(tb)});
                }
            }
            _walls.push_back(
                {Segment3D{
                     lerp(edge.source(), edge.target(), ta),
                     lerp(edge.source(), edge.target(), tb)},
                 headroom});
        }
    }
}

std::vector<Segment3D>
WallIndex::get_near_walls(const Point3D& p, double radius, double height) const
{
    std::vector<Segment3D> result{};
    const auto q = remove_z(p);
    for(const auto& [edge, headroom] : _walls) {
        // Skip manifold-cut artifacts: walkable surface within `height` above
        // the edge means it is not a physical wall (nothing to repel from).
        if(headroom <= height) {
            continue;
        }
        // filter all edges outside the (horizontal) radius
        if(CGAL::squared_distance(q, flatten(edge)) > radius * radius) {
            continue;
        }
        // Now filter on height
        if(std::abs(height_at(edge, q) - p.z()) < height) {
            result.push_back(edge);
        }
    }
    return result;
}

bool WallIndex::is_visible(const Point3D& a, const Point3D& b, double height) const
{
    const Segment3D path3d{a, b};
    const auto path2d = flatten(path3d);
    for(const auto& [edge, headroom] : _walls) {
        // Skip manifold-cut artifacts: walkable surface within `height` above
        // the edge means it is not a physical wall (does not occlude).
        if(headroom <= height) {
            continue;
        }
        const auto crossing = CGAL::intersection(path2d, flatten(edge));
        if(!crossing) {
            continue;
        }

        const auto at = [&]() -> Point2D {
            if(const auto* pt = std::get_if<Point2D>(&*crossing)) {
                // CGAL returned point --> just return it
                return *pt;
            }
            // collinear: CGAL returned a segment
            // Note: This is possible in 3D if a wall is on a different floor, so
            //       differs in height. This should be highly unlikely though.
            //       Checking heights on midpoint in this case.
            const auto& overlap = std::get<Segment2D>(*crossing);
            return CGAL::midpoint(overlap.source(), overlap.target());
        }();
        const auto zLine = height_at(path3d, at);
        const auto zEdge = height_at(edge, at);
        if(std::abs(zEdge - zLine) < height) {
            return false;
        }
    }
    return true;
}
