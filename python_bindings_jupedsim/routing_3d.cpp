// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Geometry3D.hpp"
#include "SurfaceMeshShortestPathRoutingEngine.hpp"
#include "type_casters.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <memory>

namespace py = pybind11;

void init_routing_3d(py::module_& m)
{
    // The single source of truth for a 3D geometry: mesh + projection + region
    // overlay. Fed to a routing engine and read by the viewer, so both agree.
    py::class_<Geometry3D>(m, "Geometry3D")
        .def(py::init<>())
        .def("initialize_from_obj", &Geometry3D::initialize_from_obj, py::arg("path"))
        .def("is_valid_location", &Geometry3D::is_valid_location)
        .def("region_count", &Geometry3D::region_count)
        .def("region_ids", &Geometry3D::region_ids)
        .def("vertices", &Geometry3D::vertices)
        .def("triangles", &Geometry3D::triangles);

    py::class_<RoutingEngine3D>(m, "RoutingEngine3D")
        .def("is_valid_location", &RoutingEngine3D::IsValidLocation)
        .def("get_shortest_path", &RoutingEngine3D::GetShortestPath)
        .def("get_orientation", &RoutingEngine3D::GetOrientation);

    py::class_<SurfaceMeshShortestPathRoutingEngine, RoutingEngine3D>(
        m, "SurfaceMeshShortestPathRoutingEngine")
        // The engine borrows the geometry; keep_alive ties the Python-side
        // Geometry3D's lifetime to the engine so the borrow can't dangle.
        .def(
            py::init([](const Geometry3D& geometry) {
                return std::make_unique<SurfaceMeshShortestPathRoutingEngine>(geometry);
            }),
            py::arg("geometry"),
            py::keep_alive<1, 2>());
}
