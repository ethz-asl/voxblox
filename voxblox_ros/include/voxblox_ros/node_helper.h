#include <rclcpp/rclcpp.hpp>

#pragma once

namespace node_helper {

template <typename ParameterT>
ParameterT declare_or_get_parameter(const rclcpp::Node::SharedPtr& node,
                                    const std::string& parameter_name,
                                    const ParameterT& default_value) {
  if (!node->has_parameter(parameter_name)) {
    return node->declare_parameter(parameter_name, default_value);
  }
  ParameterT parameter_value;
  node->get_parameter(parameter_name, parameter_value);
  return parameter_value;
}

}  // namespace node_helper
