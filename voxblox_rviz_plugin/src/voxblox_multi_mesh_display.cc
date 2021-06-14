#include "voxblox_rviz_plugin/voxblox_multi_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include "voxblox_rviz_plugin/material_loader.h"

namespace voxblox_rviz_plugin {

VoxbloxMultiMeshDisplay::VoxbloxMultiMeshDisplay()
    : toggle_visibility_all_property_("Toggle Visibility All", true,
                                      "Set the visibility for all meshes.",
                                      this, SLOT(toggleVisibilityAllSLOT())),
      dt_since_last_update_(0.f) {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
  // Initialize the top level of the visibility hierarchy.
  visibility_fields_.reset(new VisibilityField("Visible", this, this));
}

void VoxbloxMultiMeshDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void VoxbloxMultiMeshDisplay::visibleSlot() { updateVisible(); }

void VoxbloxMultiMeshDisplay::updateVisible() {
  // Set visibility of all visuals and update poses if visibility is turned on.
  for (auto& ns_visual_pair : visuals_) {
    bool visible = false;
    if (isEnabled()) {
      visible = visibility_fields_->isEnabled(ns_visual_pair.first);
    }
    ns_visual_pair.second.setEnabled(visible);
    if (visible) {
      updateTransformation(&(ns_visual_pair.second), ros::Time::now());
    }
  }
}

void VoxbloxMultiMeshDisplay::toggleVisibilityAllSLOT() {
  // Toggle all visibility fields except for the root.
  const bool root_visible = visibility_fields_->getBool();
  visibility_fields_->setEnabledForAll(
      toggle_visibility_all_property_.getBool());
  visibility_fields_->setBool(root_visible);
  updateVisible();
}

void VoxbloxMultiMeshDisplay::processMessage(
    const voxblox_msgs::MultiMesh::ConstPtr& msg) {
  // Select the matching visual
  auto it = visuals_.find(msg->name_space);
  if (msg->mesh.mesh_blocks.empty()) {
    // if blocks are empty the visual is to be cleared.
    if (it != visuals_.end()) {
      visibility_fields_->removeField(it->first);
      visuals_.erase(it);
    }
  } else {
    // create a visual if it does not yet exist.
    if (it == visuals_.end()) {
      it = visuals_
               .insert(std::make_pair(
                   msg->name_space,
                   VoxbloxMeshVisual(context_->getSceneManager(), scene_node_)))
               .first;
      visibility_fields_->addField(msg->name_space);
      it->second.setEnabled(visibility_fields_->isEnabled(msg->name_space));
    }

    // update the frame, pose and mesh of the visual.
    it->second.setFrameId(msg->header.frame_id);
    if (updateTransformation(&(it->second), msg->header.stamp)) {
      // here we use the multi-mesh msg header.
      // catch uninitialized alpha values, since nobody wants to display a
      // completely invisible mesh.
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
}

bool VoxbloxMultiMeshDisplay::updateTransformation(VoxbloxMeshVisual* visual,
                                                   ros::Time stamp) {
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(visual->getFrameId(), stamp,
                                                 position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              visual->getFrameId().c_str(), qPrintable(fixed_frame_));
    return false;
  }
  visual->setPose(position, orientation);
  return true;
}

void VoxbloxMultiMeshDisplay::update(float wall_dt, float ros_dt) {
  constexpr float kMinUpdateDt = 1e-1;
  dt_since_last_update_ += wall_dt;
  if (isEnabled() && kMinUpdateDt < dt_since_last_update_) {
    dt_since_last_update_ = 0;
    updateAllTransformations();
  }
}

void VoxbloxMultiMeshDisplay::updateAllTransformations() {
  for (auto& visual : visuals_) {
    updateTransformation(&(visual.second), ros::Time::now());
  }
}

void VoxbloxMultiMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  updateAllTransformations();
}

void VoxbloxMultiMeshDisplay::subscribe() {
  // Override this to allow for custom queue size, the rest is taken from
  // rviz::MessageFilterDisplay.
  if (!isEnabled()) {
    return;
  }
  try {
    ros::TransportHints transport_hint = ros::TransportHints().reliable();
    // Determine UDP vs TCP transport for user selection.
    if (unreliable_property_->getBool()) {
      transport_hint = ros::TransportHints().unreliable();
    }
    sub_.subscribe(update_nh_, topic_property_->getTopicStd(),
                   kSubscriberQueueLength, transport_hint);
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  } catch (ros::Exception& e) {
    setStatus(rviz::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  }
}

void VoxbloxMultiMeshDisplay::onInitialize() {
  // Override this to allow for custom queue size, the rest is taken from
  // rviz::MessageFilterDisplay.
  MessageFilterDisplay::onInitialize();
  tf_filter_->setQueueSize(kSubscriberQueueLength);
}

VisibilityField::VisibilityField(const std::string& name,
                                 rviz::BoolProperty* parent,
                                 VoxbloxMultiMeshDisplay* master)
    : rviz::BoolProperty(
          name.c_str(), true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          parent, SLOT(visibleSlot())),
      master_(master) {
  setDisableChildrenIfFalse(true);
}

void VisibilityField::visibleSlot() { master_->updateVisible(); }

bool VisibilityField::hasNameSpace(const std::string& name, std::string* ns,
                                   std::string* sub_name) {
  std::size_t ns_indicator = name.find('/');
  if (ns_indicator != std::string::npos) {
    *sub_name = name.substr(ns_indicator + 1);
    *ns = name.substr(0, ns_indicator);
    return true;
  }
  *sub_name = name;
  return false;
}

void VisibilityField::addField(const std::string& field_name) {
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      // Add the new namespace.
      it = children_
               .insert(std::make_pair(ns, std::unique_ptr<VisibilityField>()))
               .first;
      it->second.reset(new VisibilityField(ns, this, master_));
    }
    it->second->addField(sub_name);
  } else {
    auto it = children_
                  .insert(std::make_pair(field_name,
                                         std::unique_ptr<VisibilityField>()))
                  .first;
    it->second.reset(new VisibilityField(field_name, this, master_));
  }
}

void VisibilityField::removeField(const std::string& field_name) {
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it != children_.end()) {
      it->second->removeField(sub_name);
      if (it->second->children_.empty()) {
        // If the namespace has no more members remove it.
        children_.erase(it);
      }
    }
  } else {
    children_.erase(field_name);
  }
}

bool VisibilityField::isEnabled(const std::string& field_name) {
  if (!getBool()) {
    // This property and therefore all children are disabled.
    return false;
  }
  std::string sub_name;
  std::string ns;
  if (hasNameSpace(field_name, &ns, &sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      return false;
    }
    return it->second->isEnabled(sub_name);
  } else {
    auto it = children_.find(field_name);
    if (it == children_.end()) {
      return false;
    }
    return it->second->getBool();
  }
}

void VisibilityField::setEnabledForAll(bool enabled) {
  // Recursively set all properties.
  setBool(enabled);
  for (auto& child : children_) {
    child.second->setEnabledForAll(enabled);
  }
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMultiMeshDisplay,
                       rviz::Display)
