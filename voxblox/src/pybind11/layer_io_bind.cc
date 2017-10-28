#include "voxblox/io/layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

void layer_io_bind(py::module &m) {
  m.def("loadTsdfLayer", [](const std::string &file) {
    if (file.empty()) {
      throw std::runtime_error(std::string("Empty file path: ") + file);
    }
    TsdfLayer::Ptr layer_from_file;
    if (!voxblox::io::LoadLayer<TsdfVoxel>(file, &layer_from_file)) {
      throw std::runtime_error(std::string("Could not load layer from: ") +
                               file);
    }
    return layer_from_file;
  });

  m.def("loadEsdfLayer", [](const std::string &file) {
    if (file.empty()) {
      throw std::runtime_error(std::string("Empty file path: ") + file);
    }
    EsdfLayer::Ptr layer_from_file;
    if (!voxblox::io::LoadLayer<EsdfVoxel>(file, &layer_from_file)) {
      throw std::runtime_error(std::string("Could not load layer from: ") +
                               file);
    }
    return layer_from_file;
  });
}
