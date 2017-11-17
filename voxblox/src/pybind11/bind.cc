#include <pybind11/pybind11.h>
namespace py = pybind11;

void layer_io_bind(py::module &);
void planning_utils_bind(py::module &);
void voxel_bind(py::module &);
void block_bind(py::module &);
void layer_bind(py::module &);
void esdf_map_bind(py::module &);
void tsdf_map_bind(py::module &);
void esdf_integrator_bind(py::module &);
void tsdf_integrator_bind(py::module &);

#include "voxblox/io/layer_io.h"
using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

PYBIND11_MODULE(voxbloxpy, m) {
  voxel_bind(m);
  layer_bind(m);
  block_bind(m);
  tsdf_map_bind(m);
  esdf_map_bind(m);
  tsdf_integrator_bind(m);
  esdf_integrator_bind(m);
  layer_io_bind(m);
  planning_utils_bind(m);
}
