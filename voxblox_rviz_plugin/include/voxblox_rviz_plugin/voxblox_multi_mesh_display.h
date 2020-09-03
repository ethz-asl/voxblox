#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_

#include <map>
#include <memory>
#include <unordered_map>

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
  virtual ~VoxbloxMultiMeshDisplay() = default;

 protected:
  void reset() override;

  // override these such that not everything will be reset all the time.
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  // override subscribe to enable a custom queue size of more than 10
  static constexpr size_t kSubscriberQueueLength = 200;
  void subscribe() override;

  // Automatically update the mesh poses based on their frame_ids
  void update(float wall_dt, float ros_dt) override;

 private:
  void processMessage(const voxblox_msgs::MultiMesh::ConstPtr& msg) override;
  bool updateTransformation(VoxbloxMeshVisual* visual, ros::Time stamp);
  void updateAllTransformations();

  // The set of all visuals, identified by namespace.
  std::unordered_map<std::string, VoxbloxMeshVisual> visuals_;

  // Allows the user to still clear the mesh by clicking this property
  rviz::BoolProperty reset_property_;
  Q_SLOT void resetSlot();

  // Keep track of the time that elapsed since we last updated the submap poses,
  // such that we can throttle these updates to a reasonable rate
  float dt_since_last_update_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
