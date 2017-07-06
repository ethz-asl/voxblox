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

        .def("getDistanceAtPosition", &EsdfMap::batchGetDistanceAtPosition)
        .def("getDistanceAndGradientAtPosition", &EsdfMap::batchGetDistanceAndGradientAtPosition)
        .def("isObserved", &EsdfMap::batchIsObserved)

        .def("coordPlaneSliceGetDistance", &EsdfMap::coordPlaneSliceGetDistance,
             "Evaluate distances at all allocated voxels along an axis-aligned slice",
             py::arg("free_plane_index"),
             py::arg("free_plane_val"),
             py::arg("positions"),
             py::arg("distances"),
             py::arg("max_points") = 100000)

        ;
}
