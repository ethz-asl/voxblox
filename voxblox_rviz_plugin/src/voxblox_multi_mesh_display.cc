#include "voxblox_rviz_plugin/voxblox_multi_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMultiMeshDisplay::VoxbloxMultiMeshDisplay() :
    reset_property_("Reset Mesh",
                    false,
                    "Tick or un-tick this field to reset the mesh visualization.",
                    this,
                    SLOT(resetSlot())) {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMultiMeshDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void VoxbloxMultiMeshDisplay::resetSlot() {
  reset();
}

void VoxbloxMultiMeshDisplay::processMessage(
    const voxblox_msgs::MultiMesh::ConstPtr &msg) {
  // Select the matching visual
  auto it = visuals_.find(msg->id);
  if (it == visuals_.end()) {
    it = visuals_.insert(std::make_pair(msg->id, VoxbloxMeshVisual(context_->getSceneManager(), scene_node_))).first;
  }

  // update the frame, pose and mesh of the visual
  it->second.setFrameId(msg->header.frame_id);
  if (updateTransformation(&(it->second), msg->header.stamp)) { // here we use the multi-mesh msg header
    // catch uninitialized alpha values, since nobody wants to display a completely invisible mesh.
    uint8_t alpha = msg->alpha;
    if (alpha == 0) {
      alpha = std::numeric_limits<uint8_t>::max();
    }

    // convert to normal mesh msg for visual
    voxblox_msgs::MeshPtr mesh(new voxblox_msgs::Mesh);
    *mesh = msg->mesh;
    it->second.setMessage(mesh, alpha);
  }
}

bool VoxbloxMultiMeshDisplay::updateTransformation(VoxbloxMeshVisual *visual, ros::Time stamp) {
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(visual->getFrameId(), stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              visual->getFrameId().c_str(), qPrintable(fixed_frame_));
    return false;
  }
  visual->setPose(position, orientation);
  return true;
}

void VoxbloxMultiMeshDisplay::onDisable() {
  // Because the voxblox mesh is incremental we keep building it but don't render it.
  for (auto &visual : visuals_) {
    visual.second.setEnabled(false);
  }
}

void VoxbloxMultiMeshDisplay::onEnable() {
  for (auto &visual : visuals_) {
    visual.second.setEnabled(true);
  }
}

void VoxbloxMultiMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  for (auto &visual : visuals_) {
    updateTransformation(&(visual.second), ros::Time::now());
  }
}

void VoxbloxMultiMeshDisplay::subscribe() {
  // override this to allow for custom queue size, the rest is taken from rviz::MessageFilterDisplay
  constexpr int queue_size = 200;
  try {
    ros::TransportHints transport_hint = ros::TransportHints().reliable();
    // Determine UDP vs TCP transport for user selection.
    if (unreliable_property_->getBool()) {
      transport_hint = ros::TransportHints().unreliable();
    }
    sub_.subscribe(update_nh_, topic_property_->getTopicStd(), queue_size, transport_hint);
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  } catch (ros::Exception &e) {
    setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMultiMeshDisplay, rviz::Display)
