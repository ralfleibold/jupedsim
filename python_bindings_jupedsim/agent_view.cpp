// SPDX-License-Identifier: LGPL-3.0-or-later
#include "AgentView.hpp"
#include "EnvironmentQuery.hpp"
#include "LineSegment.hpp"
#include "OperationalModels/CustomModel/CustomModel.hpp"
#include "conversion.hpp"
#include "python_model.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <tuple>
#include <variant>

namespace py = pybind11;

void init_agent_view(py::module_& m)
{
    py::class_<NeighborView>(m, "NeighborView")
        .def_property_readonly(
            "relative_position",
            [](const NeighborView& self) { return intoTuple(self.relative_position); })
        .def_property_readonly("state", [](const NeighborView& self) -> py::object {
            const auto& state = static_cast<const OperationalModelStateVariant&>(*self.state);
            if(const auto* custom = std::get_if<CustomModel::State>(&state)) {
                // Custom states are unwrapped so that the _CustomModelState transport type
                // never reaches user code.
                return custom->Get<GilSafePyObject>().Get();
            }
            return py::cast(state);
        });

    py::class_<AgentView>(m, "AgentView")
        .def(
            "other_agents_in_range",
            [](const AgentView& self, double radius) { return self.other_agents_in_range(radius); },
            py::arg("radius"),
            "Neighbors within radius, each as the vector pointing to it.")
        .def(
            "no_geometry_between",
            [](const AgentView& self, std::tuple<double, double> relative_position) {
                return self.no_geometry_between(intoPoint(relative_position));
            },
            py::arg("relative_position"),
            "True when nothing blocks the straight line to relative_position.")
        .def(
            "inside_geometry",
            [](const AgentView& self, std::tuple<double, double> relative_position) {
                return self.inside_geometry(intoPoint(relative_position));
            },
            py::arg("relative_position"),
            "True when moving by relative_position stays inside the walkable area.")
        .def(
            "walls_nearby",
            [](const AgentView& self) { return intoVec(self.walls_nearby()); },
            "Geometry segments in the grid cells around the agent, relative to it.")
        .def(
            "walls_in_range",
            [](const AgentView& self, double distance) {
                return intoVec(self.walls_in_range(distance));
            },
            py::arg("distance"),
            "Geometry segments within exact distance of the agent, relative to it.");

    py::class_<AgentStep, AgentView>(m, "AgentStep")
        .def_property_readonly("dt", &AgentStep::dt)
        .def_property_readonly("to_next_target", [](const AgentStep& self) {
            return intoTuple(self.to_next_target());
        });
}
