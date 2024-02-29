#include "voxblox_ros/transformer.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>

#include "voxblox_ros/node_helper.h"
#include "voxblox_ros/ros_parameters.hpp"

namespace voxblox {

Transformer::Transformer(rclcpp::Node::SharedPtr node)
    : node_(node),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000) {
  world_frame_ =
      node_helper::declare_or_get_parameter(node_, "world_frame", world_frame_);
  sensor_frame_ = node_->declare_parameter("sensor_frame", sensor_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  timestamp_tolerance_sec = node_->declare_parameter("timestamp_tolerance_sec",
                                                     timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  // Transform settings.
  use_tf_transforms_ =
      node_->declare_parameter("use_tf_transforms", use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ =
        node_->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform", 40,
            std::bind(&Transformer::transformCallback, this,
                      std::placeholders::_1));

    // TODO: fix the transform parameter read
    get_transformation_parameter("T_B_D", "invert_T_B_D", T_B_D_);
    get_transformation_parameter("T_B_C", "invert_T_B_C", T_B_C_);
  }
}

void Transformer::get_transformation_parameter(
    std::string transformation_parameter_name,
    std::string invert_parameter_name, Transformation& transformation) {
  transformation = ros_parameters::get_parameter_as_transformation(
      node_, transformation_parameter_name);
  bool invert_T_B_D = node_->declare_parameter(invert_parameter_name, false);
  if (invert_T_B_D) {
    transformation = transformation.inverse();
  }
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg) {
  transform_queue_.push_back(*transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const rclcpp::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(timestamp, transform);
  }
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const rclcpp::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  geometry_msgs::msg::TransformStamped transform_msg;
  rclcpp::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_buffer_->canTransform(to_frame, from_frame_modified,
                                time_to_lookup)) {
    return false;
  }

  try {
    transform_msg = tf_buffer_->lookupTransform(to_frame, from_frame_modified,
                                                time_to_lookup);
  } catch (tf2::TransformException& ex) {  // NOLINT
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf2::Transform tf_transform;
  tf2::fromMsg(transform_msg.transform, tf_transform);
  tf2::transformTFToKindr(tf_transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const rclcpp::Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 30,
                                "No match found for transform timestamp: "
                                    << timestamp.nanoseconds()
                                    << " as transform queue is empty.");
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::msg::TransformStamped>::iterator it =
      transform_queue_.begin();
  rclcpp::Time transform_time;

  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    transform_time = it->header.stamp;
    if (transform_time > timestamp) {
      if ((transform_time - timestamp).nanoseconds() <
          timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - transform_time).nanoseconds() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    tf2::transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *(node_->get_clock()), 30,
          "No match found for transform timestamp: "
              << timestamp.nanoseconds() << " Queue front: s: "
              << transform_queue_.front().header.stamp.sec
              << " Queue front: ns: "
              << transform_queue_.front().header.stamp.nanosec
              << " back: s: " << transform_queue_.back().header.stamp.sec
              << " back: ns: " << transform_queue_.back().header.stamp.nanosec);
      return false;
    }

    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_newest);
    transform_time = it->header.stamp;
    int64_t offset_newest_ns = (transform_time - timestamp).nanoseconds();

    // We already checked that node_ is not the beginning.
    it--;
    transform_time = it->header.stamp;

    Transformation T_G_D_oldest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_oldest);
    int64_t offset_oldest_ns = (timestamp - transform_time).nanoseconds();

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  // If we have a static transform, apply it too.
  // Transform should actually be T_G_C. So need to take it through the full
  // chain.
  *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

  // And also clear the queue up to node_ point. Node_ leaves the current
  // message in place.
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
