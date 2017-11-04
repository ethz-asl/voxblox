#include "voxblox/utils/planning_utils.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using voxblox::TsdfVoxel;
using voxblox::FloatingPoint;
using voxblox::Point;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using voxblox::utils::fillSphereAroundPoint;
using voxblox::utils::clearSphereAroundPoint;

void planning_utils_bind(py::module &m) {
  m.def("fillSphereAroundPoint",
    [](TsdfLayer* layer, const Point& center, const FloatingPoint radius,
      const FloatingPoint weight) {
        if (!layer) {
          throw std::runtime_error(std::string("Empty layer"));
        }
        fillSphereAroundPoint(layer, center, radius, weight);
      }
  );
  m.def("clearSphereAroundPoint",
    [](TsdfLayer* layer, const Point& center, const FloatingPoint radius,
      const FloatingPoint weight) {
        if (!layer) {
          throw std::runtime_error(std::string("Empty layer"));
        }
        clearSphereAroundPoint(layer, center, radius, weight);
      }
  );
}
