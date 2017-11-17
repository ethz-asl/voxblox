#include <pybind11/pybind11.h>
namespace py = pybind11;

void layer_bind(py::module &);
void tango_layer_interface_bind(py::module &);
void tango_layer_io_bind(py::module &);

PYBIND11_MODULE(voxblox_tango_interfacepy, m) {
  // NOTE(mereweth@jpl.nasa.gov) - this could come from voxblox
  layer_bind(m);

  tango_layer_interface_bind(m);
  tango_layer_io_bind(m);
}
