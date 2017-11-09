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

void planning_utils_bind(py::module &m) {
  m.def("fillSphereAroundPoint",
    [](EsdfLayer* layer, const Point& center, const FloatingPoint radius) {
        if (!layer) {
          throw std::runtime_error(std::string("Empty layer"));
        }
        fillSphereAroundPoint(layer, center, radius);
      }
  );
  m.def("clearSphereAroundPoint",
    [](EsdfLayer* layer, const Point& center, const FloatingPoint radius) {
        if (!layer) {
          throw std::runtime_error(std::string("Empty layer"));
        }
        clearSphereAroundPoint(layer, center, radius);
      }
  );
}
