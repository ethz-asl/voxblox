#include "voxblox/core/esdf_map.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::EsdfMap;

PYBIND11_MODULE(voxbloxpy, m) {
  py::class_<EsdfMap>(m, "EsdfMap")
      .def(py::init<const std::string &>())
      .def_property_readonly("block_size", &EsdfMap::block_size)
      .def_property_readonly("voxel_size", &EsdfMap::voxel_size)

      .def("getDistanceAtPosition",
           &voxblox::EsdfMap::batchGetDistanceAtPosition)
      .def("getDistanceAndGradientAtPosition",
           &voxblox::EsdfMap::batchGetDistanceAndGradientAtPosition)
      .def("isObserved", &voxblox::EsdfMap::batchIsObserved)

      .def("coordPlaneSliceGetDistance",
           &voxblox::EsdfMap::coordPlaneSliceGetDistance);
}
