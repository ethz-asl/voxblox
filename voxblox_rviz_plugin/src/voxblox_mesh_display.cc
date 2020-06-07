#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() : reset_property_("Reset Mesh",
                                                           false,
                                                           "Tick or un-tick this field to reset the mesh visualization.",
                                                           this,
                                                           SLOT(resetSlot())) {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::resetSlot() {
  reset();
}

void VoxbloxMeshDisplay::processMessage(
    const voxblox_msgs::Mesh::ConstPtr &msg) {
  if (!visual_) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // update the frame, pose and mesh of the visual
  visual_->setFrameId(msg->header.frame_id);
  if (updateTransformation(msg->header.stamp)) {
    visual_->setMessage(msg);
  }
}

bool VoxbloxMeshDisplay::updateTransformation(ros::Time stamp) {
  if (!visual_) {
    // can not get the transform if we don't have a visual
    return false;
  }
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(visual_->getFrameId(), stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              visual_->getFrameId().c_str(), qPrintable(fixed_frame_));
    return false;
  }
  visual_->setPose(position, orientation);
  return true;
}

void VoxbloxMeshDisplay::onDisable() {
  // Because the voxblox mesh is incremental we keep building it but don't render it.
  if (visual_) {
    visual_->setEnabled(false);
  }
}

void VoxbloxMeshDisplay::onEnable() {
  if (visual_) {
    visual_->setEnabled(true);
  }
}

void VoxbloxMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  updateTransformation(ros::Time::now());
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay, rviz::Display)
