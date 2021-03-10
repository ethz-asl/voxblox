#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_

#include <limits>
#include <map>
#include <string>

#include <OGRE/OgreManualObject.h>

#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MultiMesh.h>

namespace voxblox_rviz_plugin {

/// Visualizes a single voxblox_msgs::Mesh message.
class VoxbloxMeshVisual {
 public:
  VoxbloxMeshVisual(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node, std::string name_space = "");
  virtual ~VoxbloxMeshVisual();

  void setMessage(const voxblox_msgs::Mesh::ConstPtr& msg,
                  uint8_t alpha = std::numeric_limits<uint8_t>::max());

  // enable / disable visibility
  void setEnabled(bool enabled);

  /// Set the coordinate frame pose.
  void setPose(const Ogre::Vector3& position,
               const Ogre::Quaternion& orientation);

  void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
  const std::string& getFrameId() { return frame_id_; }

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;
  std::string name_space_;  // this is the id used by multi-mesh messages
  bool is_enabled_;
  std::string frame_id_;  // the frame this mesh is in, newer messages will
                          // overwrite this

  voxblox::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
