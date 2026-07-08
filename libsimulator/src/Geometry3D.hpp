// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "Point.hpp"
#include "RegionSplit.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

/// The single source of truth for a 3D navigation geometry: owns the surface
/// mesh, its AABB tree (for -z projection queries) and the single-valued region
/// overlay. Routing engines borrow it (non-owning), and the viewer reads its
/// render data from here -- so mesh, routing and colouring all agree by
/// construction (one load, one face order).
class Geometry3D
{
public:
    /// Result of projecting a query point onto the surface along -z.
    struct FaceLocation {
        SurfaceMesh::Face_index face;
        SurfaceKernel::Point_3 point;
    };

    Geometry3D() = default;
    ~Geometry3D() = default;

    // Non-copyable and non-movable: the AABB tree and region property map
    // reference _mesh by address.
    Geometry3D(const Geometry3D&) = delete;
    Geometry3D& operator=(const Geometry3D&) = delete;
    Geometry3D(Geometry3D&&) = delete;
    Geometry3D& operator=(Geometry3D&&) = delete;

    /// Load geometry from a mesh file (currently OBJ; triangulated if needed).
    /// Additional formats can be added as further initialize_from_* overloads.
    void initialize_from_obj(const std::string& path);

    /// Take an already-built surface mesh (e.g. from a mesh builder or a test).
    void initialize_from_mesh(SurfaceMesh&& mesh);

    const SurfaceMesh& mesh() const { return _mesh; }
    const AABBTree& aabb_tree() const;

    /// Face and on-surface point hit by the -z ray through @p p, or
    /// `null_face()` if the ray misses the walkable surface.
    FaceLocation face_below(const Point3D& p) const;

    /// Locate @p xy within region @p region_id: the face of that region whose
    /// (x,y)-projection contains @p xy, and the on-surface point (its z on that
    /// face's plane). `null_face()` if @p xy is outside the region's footprint.
    FaceLocation locate_in_region(std::size_t region_id, const Point2D& xy) const;

    /// True iff @p p projects (along -z) onto the walkable surface.
    bool is_valid_location(const Point3D& p) const;

    // -- region overlay & render data (see split_into_regions) --------------

    std::size_t region_count() const { return _regionCount; }

    /// One 0-based region id per triangle, in mesh face order.
    std::vector<std::size_t> region_ids() const;

    /// Vertex coordinates (x, y, z), indexable 0..n-1.
    std::vector<std::array<double, 3>> vertices() const;

    /// Triangles as vertex-index triples, matching region_ids() order.
    std::vector<std::array<std::size_t, 3>> triangles() const;

private:
    /// Compact indices, build the AABB tree and the region overlay.
    void build();

    SurfaceMesh _mesh{};
    std::unique_ptr<AABBTree> _aabbTree{};
    RegionMap _region{};
    std::size_t _regionCount{0};
};
