#include "voxblox_tango_interface/core/tango_layer_interface.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TangoLayerInterface;

PYBIND11_MODULE(voxblox_tango_interfacepy, m) {
  /* NOTE(mereweth@jpl.nasa.gov) - use shared_ptr instead of default unique_ptr
   * for Python reference counting
   */
    py::class_<TangoLayerInterface, std::shared_ptr<TangoLayerInterface> >(m, "TangoLayerInterface")
        .def(py::init<const tsdf2::MapHeaderProto&>())

        .def_property_readonly("block_size", &TangoLayerInterface::block_size)
        .def_property_readonly("voxel_size", &TangoLayerInterface::voxel_size)

        .def("saveToFile", &TangoLayerInterface::saveToFile)

        ;
}
