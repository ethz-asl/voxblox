#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_

#include <memory>

#include <rviz/message_filter_display.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMeshDisplay();
  virtual ~VoxbloxMeshDisplay() = default;

 protected:
  void reset() override;
  void fixedFrameChanged() override;

 private:
  void processMessage(const voxblox_msgs::Mesh::ConstPtr& msg) override;
  bool updateTransformation(ros::Time stamp);

  std::unique_ptr<VoxbloxMeshVisual> visual_;

  // Allows the user to still clear the mesh by clicking this property
  rviz::BoolProperty visible_property_;
  Q_SLOT void visibleSLOT();
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
