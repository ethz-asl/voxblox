#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <voxblox_msgs/Mesh.h>

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::Mesh> {
  Q_OBJECT
 public:
  VoxbloxMeshDisplay();
  virtual ~VoxbloxMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const voxblox_msgs::Mesh::ConstPtr& msg);

  std::unique_ptr<VoxbloxMeshVisual> visual_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H
