#include "voxblox/core/layer.h"
#include "voxblox/core/tsdf_map.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfMap;
using voxblox::TsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

void tsdf_map_bind(py::module &m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr or default unique_ptr for
   * Python reference counting?
   */

  py::class_<TsdfMap::Config>(m, "TsdfMapConfig")
      .def(py::init<>())
      .def_readwrite("voxel_size", &TsdfMap::Config::tsdf_voxel_size)
      .def_readwrite("voxels_per_side", &TsdfMap::Config::tsdf_voxels_per_side);

  py::class_<TsdfMap, std::shared_ptr<TsdfMap> >(m, "TsdfMap")
      .def(py::init<const TsdfMap::Config &>())
      .def(py::init<TsdfLayer::Ptr>())
      .def_property_readonly("block_size", &TsdfMap::block_size)
      .def_property_readonly("voxel_size", &TsdfMap::voxel_size)

      .def("getTsdfLayer", &TsdfMap::getTsdfLayer)

      .def("coordPlaneSliceGetDistanceWeight",
           &TsdfMap::coordPlaneSliceGetDistanceWeight,
           "Evaluate distances at all allocated voxels along an axis-aligned "
           "slice",
           py::arg("free_plane_index"), py::arg("free_plane_val"),
           py::arg("positions"), py::arg("distances"), py::arg("weights"),
           py::arg("max_points") = 100000u);
}
