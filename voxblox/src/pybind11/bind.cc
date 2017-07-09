#include <pybind11/pybind11.h>
namespace py = pybind11;

void layer_io_bind(py::module &);
void voxel_bind(py::module &);
void block_bind(py::module &);
void layer_bind(py::module &);
void esdf_map_bind(py::module &);
void esdf_integrator_bind(py::module &);

PYBIND11_MODULE(voxbloxpy, m) {
  voxel_bind(m);
  layer_bind(m);
  block_bind(m);
  esdf_map_bind(m);
  esdf_integrator_bind(m);
  layer_io_bind(m);
}
