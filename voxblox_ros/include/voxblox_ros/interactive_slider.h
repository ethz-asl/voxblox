#ifndef VOXBLOX_ROS_INTERACTIVE_SLIDER_H_
#define VOXBLOX_ROS_INTERACTIVE_SLIDER_H_

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <voxblox/core/common.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace voxblox {

/// InteractiveSlider class which can be used for visualizing voxel map slices.
class InteractiveSlider {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InteractiveSlider(
      const std::string& slider_name,
      const std::function<void(const double& slice_level)>& slider_callback,
      const Point& initial_position, const unsigned int free_plane_index,
      const float marker_scale_meters);
  virtual ~InteractiveSlider() {}

 private:
  const unsigned int free_plane_index_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;

  /// Processes the feedback after moving the slider.
  virtual void interactiveMarkerFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr&
          feedback,
      const std::function<void(const double slice_level)>& slider_callback);
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_INTERACTIVE_SLIDER_H_
