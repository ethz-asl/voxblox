#ifndef VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
#define VOXBLOX_PYBIND_ESDF_MAP_BIND_H_

#include "voxblox/core/esdf_map.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

using voxblox::EsdfMap;

std::vector<bool> EsdfMap::batchGetDistanceAtPosition(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      std::vector<double> distances) const {
  CHECK_EQ(positions.cols(), distances.size());
  std::vector<bool> success;
  success.reserve(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    success[i] = getDistanceAtPosition(positions.col(i), &distances[i]);
  }

  return success;
}

std::vector<bool> EsdfMap::batchGetDistanceAndGradientAtPosition(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      std::vector<double> distances,
      EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& gradients) const {
  CHECK_EQ(positions.cols(), distances.size());
  std::vector<bool> success;
  success.reserve(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    Eigen::Vector3d gradient;
    success[i] = getDistanceAndGradientAtPosition(positions.col(i),
                                                  &distances[i],
                                                  &gradient);

    gradients.col(i) = gradient;
  }

  return success;
}

std::vector<bool> EsdfMap::batchIsObserved(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions) const {
  std::vector<bool> observed;
  observed.reserve(positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    observed[i] = isObserved(positions.col(i));
  }

  return observed;
}

PYBIND11_MODULE(voxbloxpy, m) {
    py::class_<EsdfMap>(m, "EsdfMap")
        .def(py::init<const std::string &>())
        .def_property_readonly("block_size", &EsdfMap::block_size)
        .def_property_readonly("voxel_size", &EsdfMap::voxel_size)

        .def("getDistanceAtPosition", &voxblox::EsdfMap::batchGetDistanceAtPosition)
        .def("getDistanceAndGradientAtPosition", &voxblox::EsdfMap::batchGetDistanceAndGradientAtPosition)
        .def("isObserved", &voxblox::EsdfMap::batchIsObserved)

        ;
}

#endif // VOXBLOX_PYBIND_ESDF_MAP_BIND_H_
