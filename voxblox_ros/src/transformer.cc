#include "voxblox_ros/transformer.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>

namespace voxblox {

Transformer::Transformer(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(10000000) {
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("sensor_frame", sensor_frame_, sensor_frame_);

  // Transform settings.
  nh_private_.param("use_tf_transforms", use_tf_transforms_,
                    use_tf_transforms_);
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ =
        nh_.subscribe("transform", 40, &Transformer::transformCallback, this);
    // Retrieve T_D_C from params.
    XmlRpc::XmlRpcValue T_B_D_xml;
    // TODO(helenol): split out into a function to avoid duplication.
    if (nh_private_.getParam("T_B_D", T_B_D_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_D_xml, &T_B_D_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_D", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    XmlRpc::XmlRpcValue T_B_C_xml;
    if (nh_private_.getParam("T_B_C", T_B_C_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_C_xml, &T_B_C_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_C", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_C_ = T_B_C_.inverse();
      }
    }
  }
}

void Transformer::transformCallback(
    const geometry_msgs::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const ros::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(from_frame, to_frame, timestamp, transform);
  }
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const ros::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  tf::StampedTransform tf_transform;
  ros::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // If this transform isn't possible at the time, then try to just look up
  // the latest (this is to work with bag files and static transform
  // publisher, etc).
  if (!tf_listener_.canTransform(to_frame, from_frame_modified,
                                 time_to_lookup)) {
    time_to_lookup = ros::Time(0);
    ROS_WARN("Using latest TF transform instead of timestamp match.");
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame_modified, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const std::string& from_frame,
                                       const std::string& to_frame,
                                       const ros::Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it->header.stamp > timestamp) {
      if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  if (match_found) {
    Transformation T_G_D;
    tf::transformMsgToKindr(it->transform, &T_G_D);

    // If we have a static transform, apply it too.
    // Transform should actually be T_G_C. So need to take it through the full
    // chain.
    *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

    // And also clear the queue up to this point. This leaves the current
    // message in place.
    transform_queue_.erase(transform_queue_.begin(), it);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        30, "No match found for transform timestamp: " << timestamp);
    if (!transform_queue_.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          30,
          "Queue front: " << transform_queue_.front().header.stamp
                          << " back: " << transform_queue_.back().header.stamp);
    }
  }
  return match_found;
}

}  // namespace voxblox
