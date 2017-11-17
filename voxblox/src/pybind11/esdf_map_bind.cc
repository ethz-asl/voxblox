#include "voxblox/core/esdf_map.h"
#include "voxblox/core/layer.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::EsdfMap;
using voxblox::EsdfVoxel;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

void esdf_map_bind(py::module &m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr or default unique_ptr for
   * Python reference counting?
   */

  py::class_<EsdfMap::Config>(m, "EsdfMapConfig")
      .def(py::init<>())
      .def_readwrite("voxel_size", &EsdfMap::Config::esdf_voxel_size)
      .def_readwrite("voxels_per_side", &EsdfMap::Config::esdf_voxels_per_side);

  py::class_<EsdfMap, std::shared_ptr<EsdfMap> >(m, "EsdfMap")
      .def(py::init<const EsdfMap::Config &>())
      .def(py::init<EsdfLayer::Ptr>())
      .def_property_readonly("block_size", &EsdfMap::block_size)
      .def_property_readonly("voxel_size", &EsdfMap::voxel_size)

      .def("getEsdfLayer", &EsdfMap::getEsdfLayer)

      .def("getDistanceAtPosition", &EsdfMap::batchGetDistanceAtPosition)
      .def("getDistanceAndGradientAtPosition",
           &EsdfMap::batchGetDistanceAndGradientAtPosition)
      .def("isObserved", &EsdfMap::batchIsObserved)

      .def("coordPlaneSliceGetDistance", &EsdfMap::coordPlaneSliceGetDistance,
           "Evaluate distances at all allocated voxels along an axis-aligned "
           "slice",
           py::arg("free_plane_index"), py::arg("free_plane_val"),
           py::arg("positions"), py::arg("distances"),
           py::arg("max_points") = 100000u);
}
