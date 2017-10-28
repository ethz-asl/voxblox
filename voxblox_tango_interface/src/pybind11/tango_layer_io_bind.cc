#include "voxblox_tango_interface/io/tango_layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TangoLayerInterface;
using voxblox::io::TangoLoadLayer;

void tango_layer_io_bind(py::module &m) {
  m.def("loadTangoLayer", [](const std::string &file, const bool audit) {
    if (file.empty()) {
      throw std::runtime_error(std::string("Empty file path: ") + file);
    }
    TangoLayerInterface::Ptr layer_from_file;
    if (!TangoLoadLayer(file, &layer_from_file, audit)) {
      throw std::runtime_error(std::string("Could not load layer from: ") +
                               file);
    }
    return *layer_from_file;
  },
  "Load Tango NTSDF layer from protobuf; convert to TSDF layer",
  py::arg("file_path"), py::arg("audit") = false);
}
