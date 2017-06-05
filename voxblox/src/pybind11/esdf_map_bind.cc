#ifndef VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
#define VOXBLOX_PYBIND_ESDF_MAP_BIND_H_

#include "voxblox/core/esdf_map.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(voxbloxpy, m) {
    py::class_<voxblox::EsdfMap>(m, "EsdfMap")
        .def(py::init<const std::string &>());
        //.def("setName", &Pet::setName)
        //.def("getName", &Pet::getName);
}

#endif // VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
