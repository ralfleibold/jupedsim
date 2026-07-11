// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CfgCgal.hpp"
#include "CollisionGeometry.hpp"
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

    /// Lift a 2D walkable area to a flat surface at z=0. Builds the same
    /// constrained Delaunay triangulation as the 2D RoutingEngine (identical
    /// constraint insertion order, no Steiner points), so the 2D and 3D
    /// pipelines run on the identical triangle set -- the basis for exact
    /// parity comparisons.
    void initialize_from_polygon(const PolyWithHoles& poly);

    const SurfaceMesh& mesh() const { return _mesh; }
    const AABBTree& aabb_tree() const;

    /// The projected 2D view of the walkable area, present iff the geometry
    /// was built from a polygon (initialize_from_polygon). 2D consumers --
    /// collision handling, position validation, 2D routing engines -- run on
    /// this view; a mesh-built geometry has none (nullptr) until deriving it
    /// from the mesh boundary is implemented.
    const CollisionGeometry* collision_geometry() const { return _collisionGeometry.get(); }

    /// Face and on-surface point hit by the -z ray through @p p, or
    /// `null_face()` if the ray misses the walkable surface.
    FaceLocation face_below(const Point3D& p) const;

    /// Locate @p xy within region @p region_id: the face of that region whose
    /// (x,y)-projection contains @p xy, and the on-surface point (its z on that
    /// face's plane). `null_face()` if @p xy is outside the region's footprint.
    FaceLocation locate_in_region(std::size_t region_id, const Point2D& xy) const;

    /// Re-anchor a horizontal move onto the 3D surface: @p from is in region
    /// @p from_region_id; return the face and on-surface point at @p to. The
    /// move is always a direct step, never around corners. Because the agent
    /// step is small relative to the triangle edge length, @p to lies in the
    /// start face or one directly touching it (its vertex 1-ring); using the
    /// surface neighbourhood -- not (x,y) alone -- selects the correct sheet
    /// where regions overlap (a floor passing under an upper storey). Throws if
    /// @p to is in none of them (step too large for the mesh resolution, or off
    /// the walkable area).
    FaceLocation
    walk_on_surface(std::size_t from_region_id, const Point2D& from, const Point2D& to) const;

    /// True iff @p p projects (along -z) onto the walkable surface.
    bool is_valid_location(const Point3D& p) const;

    // -- region overlay & render data (see split_into_regions) --------------

    std::size_t region_count() const { return _regionCount; }

    /// Region id (0-based) of a single face, as assigned by the region overlay.
    std::size_t region_of(SurfaceMesh::Face_index face) const { return _region[face]; }

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
    std::unique_ptr<CollisionGeometry> _collisionGeometry{};
    std::unique_ptr<AABBTree> _aabbTree{};
    RegionMap _region{};
    std::size_t _regionCount{0};
};
