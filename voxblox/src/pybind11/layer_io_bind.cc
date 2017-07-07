#include "voxblox/io/layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

void layer_io_bind(py::module &m) {
    // NOTE (mereweth@jpl.nasa.gov) - provide function signature for overloaded method
    m.def("loadTsdfLayer", (TsdfLayer (*)(const std::string &)) &voxblox::io::LoadLayer<TsdfVoxel>);

    m.def("loadEsdfLayer", (EsdfLayer (*)(const std::string &)) &voxblox::io::LoadLayer<EsdfVoxel>);
}
