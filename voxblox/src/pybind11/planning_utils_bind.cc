#include "voxblox/utils/planning_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::EsdfVoxel;
using voxblox::FloatingPoint;
using voxblox::Point;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;
using voxblox::utils::fillSphereAroundPoint;
using voxblox::utils::clearSphereAroundPoint;

void planning_utils_bind(py::module& m) {
  m.def("fillSphereAroundPoint",
        [](const Point& center, const FloatingPoint radius,
           const float max_distance_m, EsdfLayer* layer) {
          if (!layer) {
            throw std::runtime_error(std::string("Empty layer"));
          }
          fillSphereAroundPoint(center, radius, max_distance_m, layer);
        });
  m.def("clearSphereAroundPoint",
        [](const Point& center, const float max_distance_m,
           const FloatingPoint radius, EsdfLayer* layer) {
          if (!layer) {
            throw std::runtime_error(std::string("Empty layer"));
          }
          clearSphereAroundPoint(center, radius, max_distance_m, layer);
        });
}
