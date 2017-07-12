#include "voxblox_tango_interface/core/tango_layer_interface.h"
#include "voxblox/core/layer.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

// NOTE(mereweth@jpl.nasa.gov) - this is to get the base class functions
using voxblox::TsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using voxblox::FloatingPoint;

using voxblox::TangoLayerInterface;

void tango_layer_interface_bind(py::module & m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr or default unique_ptr for
   * Python reference counting?
   */

    // NOTE(mereweth@jpl.nasa.gov) - second template arg is to get the base class methods
    py::class_<TangoLayerInterface,
               TsdfLayer,
               std::shared_ptr<TangoLayerInterface> >(m, "TangoLayerInterface")
        .def(py::init<FloatingPoint, size_t, unsigned int, FloatingPoint>())

        .def_property_readonly("block_size", &TangoLayerInterface::block_size)
        .def_property_readonly("voxel_size", &TangoLayerInterface::voxel_size)
        .def_property_readonly("voxels_per_side", &TangoLayerInterface::voxels_per_side)

        // TODO(mereweth@jpl.nasa.gov) - can this still cause segfault?
        .def("saveToFile", &TangoLayerInterface::saveToFile);
}
