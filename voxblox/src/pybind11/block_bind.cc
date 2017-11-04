#include "voxblox/core/block.h"
#include "voxblox/core/voxel.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using voxblox::FloatingPoint;
using voxblox::Point;

using TsdfBlock = voxblox::Block<TsdfVoxel>;
using EsdfBlock = voxblox::Block<EsdfVoxel>;

void block_bind(py::module& m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr instead of default unique_ptr
   * for Python reference counting?
   */
  py::class_<TsdfBlock, std::shared_ptr<TsdfBlock> >(m, "TsdfBlock")
      .def(py::init<size_t, FloatingPoint, const Point&>())

      .def_property_readonly("block_size", &TsdfBlock::block_size)
      .def_property_readonly("num_voxels", &TsdfBlock::num_voxels)
      .def_property_readonly("voxel_size", &TsdfBlock::voxel_size)
      .def_property_readonly("voxels_per_side", &TsdfBlock::voxels_per_side)
      .def_property_readonly("origin", &TsdfBlock::origin)
      .def_property_readonly(
          "updated",
          (bool (TsdfBlock::*)(void) const) &TsdfBlock::updated)
      .def_property_readonly("has_data",
          (bool (TsdfBlock::*)(void) const) &TsdfBlock::has_data)

      .def("set_updated", &TsdfBlock::set_updated)
      .def("set_has_data", &TsdfBlock::set_has_data)
      .def("getVoxelByCoordinates",
           (TsdfVoxel * (TsdfBlock::*)(const Point& coords)) &
               TsdfBlock::getVoxelPtrByCoordinates,
           py::return_value_policy::reference_internal);

  py::class_<EsdfBlock, std::shared_ptr<EsdfBlock> >(m, "EsdfBlock")
      .def(py::init<size_t, FloatingPoint, const Point&>())

      .def_property_readonly("block_size", &EsdfBlock::block_size)
      .def_property_readonly("num_voxels", &EsdfBlock::num_voxels)
      .def_property_readonly("voxel_size", &EsdfBlock::voxel_size)
      .def_property_readonly("voxels_per_side", &EsdfBlock::voxels_per_side)
      .def_property_readonly("origin", &EsdfBlock::origin)
      .def_property_readonly(
          "updated",
          (bool (EsdfBlock::*)(void) const) &EsdfBlock::updated)
      .def_property_readonly("has_data",
          (bool (EsdfBlock::*)(void) const) &EsdfBlock::has_data)

      .def("set_updated", &EsdfBlock::set_updated)
      .def("set_has_data", &EsdfBlock::set_has_data)
      .def("getVoxelByCoordinates",
           (EsdfVoxel * (EsdfBlock::*)(const Point& coords)) &
               EsdfBlock::getVoxelPtrByCoordinates,
           py::return_value_policy::reference_internal);
}
