#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_

#include <OgreManualObject.h>

#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/msg/mesh.hpp>

namespace voxblox_rviz_plugin {

/// Visualizes a single voxblox_msgs::msg::Mesh message.
class VoxbloxMeshVisual {
 public:
  VoxbloxMeshVisual(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node);
  virtual ~VoxbloxMeshVisual();

  void setMessage(voxblox_msgs::msg::Mesh::ConstSharedPtr msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;

  voxblox::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
