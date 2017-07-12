#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/core/layer.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

using voxblox::TsdfIntegrator;

using voxblox::TsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

void tsdf_integrator_bind(py::module &m) {
    py::class_<TsdfIntegrator::Config>(m, "TsdfIntegratorConfig")
        .def(py::init<>());

    py::class_<TsdfIntegrator>(m, "TsdfIntegrator")
        .def(py::init<const TsdfIntegrator::Config &, TsdfLayer* >())

        .def("clearSphereAroundPoint", &TsdfIntegrator::clearSphereAroundPoint)
        .def("fillSphereAroundPoint", &TsdfIntegrator::fillSphereAroundPoint);
}
