#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY2_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY2_H_

#include <memory>
#include <map>

#include <rviz/message_filter_display.h>
#include <voxblox_msgs/MultiMesh.h>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMultiMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::MultiMesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMultiMeshDisplay();
  virtual ~VoxbloxMultiMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const voxblox_msgs::MultiMesh::ConstPtr& msg);

  std::unordered_map<int, VoxbloxMeshVisual> visuals_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
