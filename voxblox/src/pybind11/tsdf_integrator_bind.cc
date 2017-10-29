#include "voxblox/core/layer.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfIntegratorBase;
using voxblox::SimpleTsdfIntegrator;

using voxblox::TsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

void tsdf_integrator_bind(py::module &m) {
  py::class_<SimpleTsdfIntegrator::Config>(m, "TsdfIntegratorConfig")
      .def(py::init<>());

  py::class_<SimpleTsdfIntegrator>(m, "SimpleTsdfIntegrator")
      .def(py::init<const SimpleTsdfIntegrator::Config &, TsdfLayer *>());
}
