#include "voxblox_rviz_plugin/voxblox_multi_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>
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

  auto it = visuals_.find(msg->id);
  if (it == visuals_.end()){
    it = visuals_.insert(std::make_pair(msg->id, VoxbloxMeshVisual(context_->getSceneManager(), scene_node_->createChildSceneNode()))).first;
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
  it->second.setMessage(mesh_msg, msg->alpha);
  it->second.setFramePosition(position);
  it->second.setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMultiMeshDisplay, rviz::Display)
