#include "voxblox_rviz_plugin/voxblox_multi_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMultiMeshDisplay::VoxbloxMultiMeshDisplay() {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMultiMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

VoxbloxMultiMeshDisplay::~VoxbloxMultiMeshDisplay() {}

void VoxbloxMultiMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMultiMeshDisplay::processMessage(
    const voxblox_msgs::MultiMesh::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  if (!visual_){
    visual_.reset(  new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  voxblox_msgs::MeshPtr mesh_msg(new voxblox_msgs::Mesh());
  *mesh_msg=msg->mesh;
  mesh_msg->header = msg->header;
  uint8_t alpha = msg->alpha;
  if (alpha == 0){
    // catch uninitialized alpha values, since nobody wants to display a completely invisible mesh.
    alpha = std::numeric_limits<uint8_t>::max();
  }
  visual_->setMessage(mesh_msg, msg->alpha, msg->id);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMultiMeshDisplay, rviz::Display)
