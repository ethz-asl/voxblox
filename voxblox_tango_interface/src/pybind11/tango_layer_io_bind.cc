#include "voxblox_tango_interface/io/tango_layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::io::TangoLoadLayer;
using voxblox::io::TangoLoadOrCreateLayer;

void tango_layer_io_bind(py::module &m) {
    m.def("tangoLoadLayer", &TangoLoadLayer);

    m.def("tangoLoadOrCreateLayer", &TangoLoadOrCreateLayer);
}
