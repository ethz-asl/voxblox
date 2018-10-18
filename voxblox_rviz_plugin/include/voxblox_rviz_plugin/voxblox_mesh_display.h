#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_

#include <rviz/message_filter_display.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MinimalMesh.h>
#include <memory>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMeshDisplay();
  virtual ~VoxbloxMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const voxblox_msgs::Mesh::ConstPtr& msg);

  std::unique_ptr<VoxbloxMeshVisual> visual_;
};

// lots of copy pasted code, but the Q_OBJECT macro doesn't support templates
class VoxbloxMinimalMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::MinimalMesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMinimalMeshDisplay();
  virtual ~VoxbloxMinimalMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const voxblox_msgs::MinimalMesh::ConstPtr& msg);

  std::unique_ptr<VoxbloxMinimalMeshVisual> visual_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
