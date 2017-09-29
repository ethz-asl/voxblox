#include "voxblox/io/layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

//TODO(mereweth@jpl.nasa.gov) - why doesn't this compile anymore?
// Potential quickfix is to return Layer::Ptr, as this is compatible with Python
// reference counting
//auto loadTsdfLayer = (TsdfLayer (*)(const std::string &)) &voxblox::io::LoadLayer<TsdfVoxel>;
//auto loadEsdfLayer = (EsdfLayer (*)(const std::string &)) &voxblox::io::LoadLayer<EsdfVoxel>;

void layer_io_bind(py::module &m) {
    // NOTE (mereweth@jpl.nasa.gov) - provide function signature for overloaded method
    m.def("loadTsdfLayer", [](const std::string & file) {
      if (file.empty()) {
        throw std::runtime_error(std::string("Empty file path: ")
                                 + file);
      }
      TsdfLayer::Ptr layer_from_file;
      voxblox::io::LoadLayer<TsdfVoxel>(file, &layer_from_file);
      return layer_from_file;
    });

    m.def("loadEsdfLayer", [](const std::string & file) {
      if (file.empty()) {
        throw std::runtime_error(std::string("Empty file path: ")
                                 + file);
      }
      EsdfLayer::Ptr layer_from_file;
      voxblox::io::LoadLayer<EsdfVoxel>(file, &layer_from_file);
      return layer_from_file;
    });

    //m.def("loadEsdfLayer", loadEsdfLayer);
}
