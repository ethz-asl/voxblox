#include "voxblox/core/layer.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using voxblox::FloatingPoint;

using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

void layer_bind(py::module &m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr instead of default unique_ptr
   * for Python reference counting?
   */
  py::class_<TsdfLayer, std::shared_ptr<TsdfLayer> >(m, "TsdfLayer")
      .def(py::init<FloatingPoint, size_t>())

      .def_property_readonly("block_size", &TsdfLayer::block_size)
      .def_property_readonly("voxel_size", &TsdfLayer::voxel_size)
      .def_property_readonly("voxels_per_side", &TsdfLayer::voxels_per_side)

      .def("saveToFile", (bool (TsdfLayer::*)(const std::string &) const) &
                             TsdfLayer::saveToFile)

      .def("allocateBlockPtrByCoordinates",
           &TsdfLayer::allocateBlockPtrByCoordinates)
      .def("removeBlockByCoordinates", &TsdfLayer::removeBlockByCoordinates);

  py::class_<EsdfLayer, std::shared_ptr<EsdfLayer> >(m, "EsdfLayer")
      .def(py::init<FloatingPoint, size_t>())

      .def_property_readonly("block_size", &EsdfLayer::block_size)
      .def_property_readonly("voxel_size", &EsdfLayer::voxel_size)
      .def_property_readonly("voxels_per_side", &EsdfLayer::voxels_per_side)

      .def("saveToFile", (bool (EsdfLayer::*)(const std::string &) const) &
                             EsdfLayer::saveToFile)
      .def("allocateBlockPtrByCoordinates",
           &EsdfLayer::allocateBlockPtrByCoordinates)
      .def("removeBlockByCoordinates", &EsdfLayer::removeBlockByCoordinates);
}
