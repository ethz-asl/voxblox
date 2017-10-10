#include "voxblox/core/voxel.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;

void voxel_bind(py::module &m) {
  /* TODO(mereweth@jpl.nasa.gov) - use shared_ptr instead of default unique_ptr
   * for Python reference counting?
   */
  py::class_<TsdfVoxel>(m, "TsdfVoxel")
      .def(py::init<>())

      .def_readwrite("distance", &TsdfVoxel::distance)
      .def_readwrite("weight", &TsdfVoxel::weight);

  py::class_<EsdfVoxel>(m, "EsdfVoxel")
      .def(py::init<>())

      .def_readwrite("distance", &EsdfVoxel::distance)
      .def_readwrite("observed", &EsdfVoxel::observed)
      .def_readwrite("in_queue", &EsdfVoxel::in_queue)
      .def_readwrite("fixed", &EsdfVoxel::fixed)
      .def_readwrite("parent", &EsdfVoxel::parent);
}
