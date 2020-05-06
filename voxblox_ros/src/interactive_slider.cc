#include "voxblox_ros/interactive_slider.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/Marker.h>

namespace voxblox {

InteractiveSlider::InteractiveSlider(
    const std::string& slider_name,
    const std::function<void(const double& slice_level)>& slider_callback,
    const Point& initial_position, const unsigned int free_plane_index,
    const float marker_scale_meters)
    : free_plane_index_(free_plane_index),
      interactive_marker_server_(slider_name) {
  CHECK(!slider_name.empty());
  CHECK_LT(free_plane_index, 3u);

  // Create an interactive marker.
  visualization_msgs::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = "map";
  interactive_marker.header.stamp = ros::Time::now();
  interactive_marker.pose.position.x =
      static_cast<double>(initial_position.x());
  interactive_marker.pose.position.y =
      static_cast<double>(initial_position.y());
  interactive_marker.pose.position.z =
      static_cast<double>(initial_position.z());

  // Create a marker.
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = marker_scale_meters;
  marker.scale.y = marker_scale_meters;
  marker.scale.z = marker_scale_meters;
  constexpr float kGreyValue = 0.1f;
  marker.color.r = kGreyValue;
  marker.color.g = kGreyValue;
  marker.color.b = kGreyValue;
  marker.color.a = 1.0f;

  // Control for moving the marker in the direction of the free plane index.
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  switch (free_plane_index_) {
    case 0u:
      marker.scale.x *= 0.1f;
      control.orientation.x = 1.0;
      break;
    case 1u:
      marker.scale.y *= 0.1f;
      control.orientation.z = 1.0;
      break;
    case 2u:
      marker.scale.z *= 0.1f;
      control.orientation.y = 1.0;
  }
  interactive_marker.controls.push_back(control);

  // Control for moving the marker in the plane which is orthogonal to the free
  // plane index direction.
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back(marker);
  interactive_marker.controls.push_back(control);

  // Add interactive marker to server.
  interactive_marker_server_.insert(
      interactive_marker,
      std::bind(&InteractiveSlider::interactiveMarkerFeedback, this,
                std::placeholders::_1, slider_callback));
  interactive_marker_server_.applyChanges();

  // Initial callback.
  switch (free_plane_index_) {
    case 0u:
      slider_callback(interactive_marker.pose.position.x);
      break;
    case 1u:
      slider_callback(interactive_marker.pose.position.y);
      break;
    case 2u:
      slider_callback(interactive_marker.pose.position.z);
  }
}

void InteractiveSlider::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
    const std::function<void(const double slice_level)>& slider_callback) {
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    switch (free_plane_index_) {
      case 0u:
        slider_callback(feedback->pose.position.x);
        break;
      case 1u:
        slider_callback(feedback->pose.position.y);
        break;
      case 2u:
        slider_callback(feedback->pose.position.z);
    }
  }
}

}  // namespace voxblox
