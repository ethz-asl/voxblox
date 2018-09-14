#include "voxblox/core/layer.h"
#include "voxblox/integrator/esdf_integrator.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::EsdfIntegrator;

using voxblox::TsdfVoxel;
using voxblox::EsdfVoxel;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;

void esdf_integrator_bind(py::module &m) {  // NOLINT
  py::class_<EsdfIntegrator::Config>(m, "EsdfIntegratorConfig")
      .def(py::init<>())
      .def_readwrite("max_distance_m", &EsdfIntegrator::Config::max_distance_m)
      .def_readwrite("min_distance_m", &EsdfIntegrator::Config::min_distance_m)
      .def_readwrite("default_distance_m",
                     &EsdfIntegrator::Config::default_distance_m)
      .def_readwrite("min_weight", &EsdfIntegrator::Config::min_weight)
      .def_readwrite("num_buckets", &EsdfIntegrator::Config::num_buckets)
      .def_readwrite("clear_sphere_radius",
                     &EsdfIntegrator::Config::clear_sphere_radius)
      .def_readwrite("occupied_sphere_radius",
                     &EsdfIntegrator::Config::occupied_sphere_radius);

  py::class_<EsdfIntegrator>(m, "EsdfIntegrator")
      .def(py::init<const EsdfIntegrator::Config &, TsdfLayer *, EsdfLayer *>())

      .def("updateFromTsdfLayerBatch",
           &EsdfIntegrator::updateFromTsdfLayerBatch)
      .def("updateFromTsdfLayer", &EsdfIntegrator::updateFromTsdfLayer);
}
