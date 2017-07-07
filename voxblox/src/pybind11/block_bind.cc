#include "voxblox/core/voxel.h"
#include "voxblox/core/block.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using voxblox::BlockProto;

using TsdfBlock = voxblox::Block<TsdfVoxel>;
using EsdfBlock = voxblox::Block<EsdfVoxel>;

void block_bind(py::module &m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr instead of default unique_ptr
   * for Python reference counting?
   */
    py::class_<TsdfBlock>(m, "TsdfBlock")
    //py::class_<TsdfBlock, std::shared_ptr<TsdfBlock> >(m, "TsdfBlock")
        .def(py::init<const BlockProto&>())

        .def_property_readonly("block_size", &TsdfBlock::block_size)
        .def_property_readonly("num_voxels", &TsdfBlock::num_voxels)
        .def_property_readonly("voxel_size", &TsdfBlock::voxel_size)
        .def_property_readonly("voxels_per_side", &TsdfBlock::voxels_per_side)

        ;

    py::class_<EsdfBlock>(m, "EsdfBlock")
    //py::class_<EsdfBlock, std::shared_ptr<EsdfBlock> >(m, "EsdfBlock")
        .def(py::init<const BlockProto&>())

        .def_property_readonly("block_size", &EsdfBlock::block_size)
        .def_property_readonly("num_voxels", &TsdfBlock::num_voxels)
        .def_property_readonly("voxel_size", &EsdfBlock::voxel_size)
        .def_property_readonly("voxels_per_side", &EsdfBlock::voxels_per_side)

        ;
}
