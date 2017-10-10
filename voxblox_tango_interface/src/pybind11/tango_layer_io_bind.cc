#include "voxblox_tango_interface/io/tango_layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TangoLayerInterface;
using voxblox::io::TangoLoadLayer;

void tango_layer_io_bind(py::module &m) {
  m.def("tangoLoadLayer",
        (TangoLayerInterface(*)(const std::string &, bool)) & TangoLoadLayer,
        "Load Tango NTSDF layer from protobuf; convert to TSDF layer",
        py::arg("file_path"), py::arg("audit") = false);
}
