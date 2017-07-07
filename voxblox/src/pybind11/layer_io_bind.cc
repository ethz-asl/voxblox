#include "voxblox/io/layer_io.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;

void layer_io_bind(py::module &m) {
    m.def("loadTsdfLayer", &voxblox::io::LoadLayer<TsdfVoxel>);

    m.def("loadEsdfLayer", &voxblox::io::LoadLayer<EsdfVoxel>);

    m.def("loadOrCreateTsdfLayerHeader", &voxblox::io::LoadOrCreateLayerHeader<TsdfVoxel>);

    m.def("loadOrCreateEsdfLayerHeader", &voxblox::io::LoadOrCreateLayerHeader<EsdfVoxel>);
}
