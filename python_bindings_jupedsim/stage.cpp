// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Stage.hpp"

#include "type_casters.hpp" // IWYU pragma: keep

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

namespace py = pybind11;

void init_stage(py::module_& m)
{
    py::class_<BaseStage>(m, "BaseStage").def("count_targeting", &BaseStage::CountTargeting);
    py::class_<Waypoint, BaseStage>(m, "WaypointStage");
    py::class_<Exit, BaseStage>(m, "ExitStage");
    py::class_<DirectSteering>(m, "DirectSteeringStage");
}
