// SPDX-License-Identifier: LGPL-3.0-or-later
#include "RegionSplit.hpp"

#include <CGAL/Bbox_2.h>
#include <CGAL/intersections.h>

#include <array>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace
{
using P2 = SurfaceKernel::Point_2;
using T2 = SurfaceKernel::Triangle_2;

/// Orthogonal projection of a triangular face onto the x/y-plane (z dropped).
T2 project(const SurfaceMesh& mesh, SurfaceMesh::Face_index f)
{
    std::array<P2, 3> p{};
    int i = 0;
    for(const auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
        const auto& q = mesh.point(v);
        p[i++] = P2(q.x(), q.y());
    }
    return T2(p[0], p[1], p[2]);
}

/// True iff the two projected triangles share a positive-area region. Faces of
/// one single-valued region only ever touch along shared edges/vertices, whose
/// intersection is a segment/point -- those are deliberately NOT overlaps.
bool overlaps(const T2& a, const T2& b)
{
    const auto result = CGAL::intersection(a, b);
    if(!result) {
        return false;
    }
    if(std::get_if<T2>(&*result)) {
        return true; // triangle-shaped overlap
    }
    if(const auto* poly = std::get_if<std::vector<P2>>(&*result)) {
        return poly->size() >= 3; // convex polygon overlap
    }
    return false; // Point_2 / Segment_2: boundary touch only
}
} // namespace

RegionSplit split_into_regions(SurfaceMesh& mesh)
{
    auto [region, added] =
        mesh.add_property_map<SurfaceMesh::Face_index, std::size_t>("f:region", 0);
    static_cast<void>(added);

    constexpr auto UNASSIGNED = std::numeric_limits<std::size_t>::max();
    for(const auto f : faces(mesh)) {
        region[f] = UNASSIGNED;
    }

    std::size_t next_id = 0;
    for(const auto seed : faces(mesh)) {
        if(region[seed] != UNASSIGNED) {
            continue;
        }
        const auto id = next_id++;

        // Projected triangles already accepted into this region, each with its
        // 2D bounding box for a cheap overlap pre-filter.
        std::vector<std::pair<T2, CGAL::Bbox_2>> members{};
        std::queue<SurfaceMesh::Face_index> frontier{};
        const auto join = [&](SurfaceMesh::Face_index f) {
            region[f] = id;
            const auto t = project(mesh, f);
            members.emplace_back(t, t.bbox());
            frontier.push(f);
        };
        join(seed);

        while(!frontier.empty()) {
            const auto g = frontier.front();
            frontier.pop();
            for(const auto h : CGAL::halfedges_around_face(mesh.halfedge(g), mesh)) {
                const auto opp = mesh.opposite(h);
                if(mesh.is_border(opp)) {
                    continue;
                }
                const auto nbr = mesh.face(opp);
                if(region[nbr] != UNASSIGNED) {
                    continue;
                }
                const auto t = project(mesh, nbr);
                const auto bb = t.bbox();
                bool clashes = false;
                for(const auto& [member_t, member_bb] : members) {
                    if(CGAL::do_overlap(bb, member_bb) && overlaps(t, member_t)) {
                        clashes = true;
                        break;
                    }
                }
                // A neighbour that folds back over this region's footprint stays
                // UNASSIGNED and seeds its own region on a later outer iteration.
                if(!clashes) {
                    join(nbr);
                }
            }
        }
    }
    return {region, next_id};
}
