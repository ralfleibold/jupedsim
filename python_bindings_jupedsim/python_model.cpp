// SPDX-License-Identifier: LGPL-3.0-or-later
#include "python_model.hpp"

#include "GenericAgent.hpp"
#include "InformationForUpdate.hpp"
#include "LineSegment.hpp"
#include "OperationalModel.hpp"
#include "OperationalModels/CustomModel/CustomModelData.hpp"
#include "OperationalModels/CustomModel/CustomModelUpdate.hpp"
#include "SimulationError.hpp"
#include "conversion.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <span>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace py = pybind11;

GilSafePyObject::GilSafePyObject(py::object obj) : _obj(std::move(obj))
{
}

GilSafePyObject::GilSafePyObject(const GilSafePyObject& other)
{
    py::gil_scoped_acquire gil;
    _obj = other._obj; // share by reference (incref), do not clone
}

GilSafePyObject& GilSafePyObject::operator=(const GilSafePyObject& other)
{
    if(this == &other) {
        return *this;
    }

    py::gil_scoped_acquire gil;
    _obj = other._obj; // share by reference (incref), do not clone
    return *this;
}

GilSafePyObject& GilSafePyObject::operator=(GilSafePyObject&& other) noexcept
{
    if(this == &other) {
        return *this;
    }

    // Move-assignment decrefs the previously held object, so it must hold the GIL.
    py::gil_scoped_acquire gil;
    _obj = std::move(other._obj);
    return *this;
}

GilSafePyObject::~GilSafePyObject()
{
    py::gil_scoped_acquire gil;
    _obj = py::object();
}

const py::object& GilSafePyObject::Get() const
{
    return _obj;
}

py::object& GilSafePyObject::Get()
{
    return _obj;
}

void GilSafePyObject::Set(py::object obj)
{
    py::gil_scoped_acquire gil;
    _obj = std::move(obj);
}

namespace
{
InformationRequirements
requirements_from_python(const py::object& model, const char* adapter, const GenericAgent& agent)
{
    py::gil_scoped_acquire gil;

    const auto [neighborRadius, wallRadius] =
        py::cast<std::tuple<std::optional<double>, std::optional<double>>>(
            model.attr(adapter)(py::cast(agent)));
    return {.neighborRadius = neighborRadius, .wallRadius = wallRadius};
}

py::list walls_as_python_list(std::span<const LineSegment> walls)
{
    py::list result{};
    for(const auto& wall : walls) {
        result.append(py::cast(wall));
    }
    return result;
}
} // namespace

PythonModel::PythonModel(py::object model) : _model(std::move(model))
{
    py::gil_scoped_acquire gil;
    if(!_model || _model.is_none()) {
        throw std::invalid_argument("_PythonModel requires a CustomOperationalModel instance");
    }
    if(!py::hasattr(_model, "_compute_new_position") ||
       !py::hasattr(_model, "_check_model_constraint") ||
       !py::hasattr(_model, "_information_requirements") ||
       !py::hasattr(_model, "_constraint_requirements")) {
        throw std::invalid_argument("_PythonModel requires a CustomOperationalModel instance");
    }
}

InformationRequirements PythonModel::Requirements(const GenericAgent& agent) const
{
    return requirements_from_python(_model, "_information_requirements", agent);
}

InformationRequirements PythonModel::ConstraintRequirements(const GenericAgent& agent) const
{
    return requirements_from_python(_model, "_constraint_requirements", agent);
}

OperationalModelUpdate PythonModel::ComputeNewPosition(
    double dT,
    const GenericAgent& agent,
    const InformationForUpdate& info) const
{
    py::gil_scoped_acquire gil;

    py::object update = _model.attr("_compute_new_position")(
        dT, py::cast(agent), py::cast(info.neighbors), walls_as_python_list(info.walls));

    return CustomModelUpdate{GilSafePyObject{std::move(update)}};
}

void PythonModel::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    py::gil_scoped_acquire gil;

    const auto& pythonUpdate = std::get<CustomModelUpdate>(update).Get<GilSafePyObject>().Get();
    auto& customModelData = std::get<CustomModelData>(agent.model).Get<GilSafePyObject>();
    if(pythonUpdate.is(customModelData.Get())) {
        throw SimulationError(
            "Current and updated model state are the same instance. "
            "compute_new_position() must return a new state object, "
            "e.g. dataclasses.replace(ped.model, ...).");
    }

    constexpr auto attr_name = "position";
    py::object attr;
    try {
        attr = pythonUpdate.attr(attr_name);
    } catch(const py::error_already_set& ex) {
        if(ex.matches(PyExc_AttributeError)) {
            throw SimulationError(
                "State returned by compute_new_position() is missing the '{}' attribute.",
                attr_name);
        }
        throw;
    }

    try {
        agent.pos = intoPoint(py::cast<std::tuple<double, double>>(attr));
    } catch(const py::cast_error&) {
        // Diagnostics run Python code on the offending object; they must not
        // be able to replace the error they describe.
        std::string actualType = "<unknown>";
        std::string valueRepr = "<unprintable>";
        try {
            actualType = std::string(py::str(py::type::of(attr).attr("__name__")));
        } catch(const py::error_already_set&) {
        }
        try {
            valueRepr = std::string(py::repr(attr));
        } catch(const py::error_already_set&) {
        }

        throw SimulationError(
            "State returned by compute_new_position() has attribute '{}' of wrong type: "
            "expected tuple[float, float], got {} ({})",
            attr_name,
            actualType,
            valueRepr);
    }
    customModelData.Set(pythonUpdate);
}

void PythonModel::CheckModelConstraint(
    const GenericAgent& agent,
    const InformationForUpdate& info) const
{
    py::gil_scoped_acquire gil;

    _model.attr("_check_model_constraint")(
        py::cast(agent), py::cast(info.neighbors), walls_as_python_list(info.walls));
}

void init_python_model(py::module_& m)
{
    py::class_<OperationalModel, py::smart_holder>(m, "OperationalModel");

    py::class_<CustomModelData>(m, "_CustomModelData")
        .def(py::init([](py::object model) {
            return CustomModelData{GilSafePyObject{std::move(model)}};
        }))
        .def_property_readonly(
            "model", [](CustomModelData& data) { return data.Get<GilSafePyObject>().Get(); });

    py::class_<PythonModel, OperationalModel, py::smart_holder>(m, "_PythonModel")
        .def(py::init<py::object>(), py::arg("model"));
}
