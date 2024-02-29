#pragma once

#include <iostream>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "voxblox_ros/transformer.h"

namespace ros_parameters {
std::vector<std::vector<double>> convert_to_matrix(
    const std::string& multilineString) {
  std::vector<std::vector<double>> result;

  std::istringstream iss(multilineString);
  std::string line;
  while (std::getline(iss, line)) {
    std::istringstream lineStream(line);
    std::vector<double> row;
    std::string valueStr;
    while (std::getline(lineStream, valueStr, ',')) {
      double value = std::stod(valueStr);
      row.push_back(value);
    }
    result.push_back(row);
  }

  return result;
}

voxblox::Transformation get_parameter_as_transformation(
    rclcpp::Node::SharedPtr node, std::string parameter_name) {
  voxblox::Transformation transformation;
  auto matrix_string = node->declare_parameter(parameter_name, "");
  if (!matrix_string.empty()) {
    std::vector<std::vector<double>> matrix =
        ros_parameters::convert_to_matrix(matrix_string);
    kindr::minimal::vectorOfVectorsToKindr(matrix, &transformation);
  }
  return transformation;
}
}  // namespace ros_parameters