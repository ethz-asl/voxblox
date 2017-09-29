#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/core/layer.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

using voxblox::TsdfIntegratorBase;
using voxblox::SimpleTsdfIntegrator;

using voxblox::TsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

void tsdf_integrator_bind(py::module &m) {
    py::class_<SimpleTsdfIntegrator::Config>(m, "TsdfIntegratorConfig")
        .def(py::init<>());

    // py::class_<TsdfIntegratorBase>(m, "TsdfIntegratorBase")
    //     .def(py::init<const TsdfIntegratorBase::Config &, TsdfLayer* >())
    //
    //     // TODO(mereweth@jpl.nasa.gov) - how to overload this per derived class?
    //     .def("clearSphereAroundPoint", &TsdfIntegratorBase::clearSphereAroundPoint)
    //     .def("fillSphereAroundPoint", &TsdfIntegratorBase::fillSphereAroundPoint);

    py::class_<SimpleTsdfIntegrator>(m, "SimpleTsdfIntegrator")
        .def(py::init<const SimpleTsdfIntegrator::Config &, TsdfLayer* >())

        // TODO(mereweth@jpl.nasa.gov) - how to overload this per derived class?
        .def("clearSphereAroundPoint", &SimpleTsdfIntegrator::clearSphereAroundPoint)
        .def("fillSphereAroundPoint", &SimpleTsdfIntegrator::fillSphereAroundPoint);
}
