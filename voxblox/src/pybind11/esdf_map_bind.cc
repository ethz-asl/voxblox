#ifndef VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
#define VOXBLOX_PYBIND_ESDF_MAP_BIND_H_

#include "voxblox/core/esdf_map.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

using voxblox::EsdfMap;

PYBIND11_MODULE(voxbloxpy, m) {
    py::class_<EsdfMap>(m, "EsdfMap")
        .def(py::init<const std::string &>())
        .def_property_readonly("block_size", &EsdfMap::block_size)
        .def_property_readonly("voxel_size", &EsdfMap::voxel_size)
/*
        .def("getDistanceAtPosition", &voxblox::EsdfMap::getDistanceAtPosition)
        .def("getDistanceAndGradientAtPosition", &voxblox::EsdfMap::getDistanceAndGradientAtPosition)
        .def("isObserved", &voxblox::EsdfMap::isObserved)
*/
        ;
}

#endif // VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
