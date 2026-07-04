// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Geometry3D.hpp"

#include "SimulationError.hpp"

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>
#include <CGAL/boost/graph/helpers.h>

#include <utility>
#include <variant>

void Geometry3D::initialize_from_obj(const std::string& path)
{
    SurfaceMesh mesh{};
    if(!CGAL::IO::read_polygon_mesh(path, mesh) || mesh.is_empty()) {
        throw SimulationError("Could not read a mesh from OBJ file '{}'", path);
    }
    if(!CGAL::is_triangle_mesh(mesh)) {
        CGAL::Polygon_mesh_processing::triangulate_faces(mesh);
    }
    initialize_from_mesh(std::move(mesh));
}

void Geometry3D::initialize_from_mesh(SurfaceMesh&& mesh)
{
    _mesh = std::move(mesh);
    build();
}

void Geometry3D::build()
{
    // Compact vertex/face indices so vertices()/triangles()/region_ids() are
    // contiguous and 1:1 with each other (triangulate_faces may leave removed
    // faces behind).
    _mesh.collect_garbage();
    _aabbTree = std::make_unique<AABBTree>(_mesh.faces().begin(), _mesh.faces().end(), _mesh);
    const auto split = split_into_regions(_mesh);
    _region = split.region;
    _regionCount = split.count;
}

const AABBTree& Geometry3D::aabb_tree() const
{
    if(!_aabbTree) {
        throw SimulationError("Geometry3D has no geometry loaded.");
    }
    return *_aabbTree;
}

Geometry3D::FaceLocation Geometry3D::face_below(const Point3D& p) const
{
    // first_intersection along -z returns the hit nearest to the ray source,
    // i.e. the face directly below the query point.
    const SurfaceKernel::Ray_3 ray(p, SurfaceKernel::Direction_3(0, 0, -1));
    const auto hit = aabb_tree().first_intersection(ray);
    if(!hit) {
        return {SurfaceMesh::null_face(), SurfaceKernel::Point_3{}};
    }
    const auto* projected = std::get_if<SurfaceKernel::Point_3>(&hit->first);
    if(!projected) {
        return {SurfaceMesh::null_face(), SurfaceKernel::Point_3{}};
    }
    return {hit->second, *projected};
}

bool Geometry3D::is_valid_location(const Point3D& p) const
{
    return face_below(p).face != SurfaceMesh::null_face();
}

std::vector<std::size_t> Geometry3D::region_ids() const
{
    std::vector<std::size_t> ids{};
    ids.reserve(_mesh.number_of_faces());
    for(const auto f : _mesh.faces()) {
        ids.push_back(_region[f]);
    }
    return ids;
}

std::vector<std::array<double, 3>> Geometry3D::vertices() const
{
    std::vector<std::array<double, 3>> out{};
    out.reserve(_mesh.number_of_vertices());
    for(const auto v : _mesh.vertices()) {
        const auto& p = _mesh.point(v);
        out.push_back({p.x(), p.y(), p.z()});
    }
    return out;
}

std::vector<std::array<std::size_t, 3>> Geometry3D::triangles() const
{
    std::vector<std::array<std::size_t, 3>> out{};
    out.reserve(_mesh.number_of_faces());
    for(const auto f : _mesh.faces()) {
        std::array<std::size_t, 3> tri{};
        int i = 0;
        for(const auto v : CGAL::vertices_around_face(_mesh.halfedge(f), _mesh)) {
            tri[i++] = static_cast<std::size_t>(v);
        }
        out.push_back(tri);
    }
    return out;
}
