#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <tf2_ros/transform_listener.h>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

VoxbloxMeshDisplay::~VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::processMessage(
    voxblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    std::string errorMessage = "Error transforming from frame '" +
                               msg->header.frame_id + "' to frame '" +
                               fixed_frame_.toStdString() + "'";
    RVIZ_COMMON_LOG_ERROR(errorMessage);
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay,
                       rviz_common::Display)
