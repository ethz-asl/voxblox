#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/RTShaderSystem/OgreRTShaderSystem.h>
#include <OGRE/RTShaderSystem/OgreShaderGenerator.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

VoxbloxMeshDisplay::~VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::processMessage(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
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

  if (visual_ == nullptr) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

VoxbloxMinimalMeshDisplay::VoxbloxMinimalMeshDisplay() {}

void VoxbloxMinimalMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

VoxbloxMinimalMeshDisplay::~VoxbloxMinimalMeshDisplay() {}

void VoxbloxMinimalMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMinimalMeshDisplay::processMessage(
    const voxblox_msgs::MinimalMesh::ConstPtr& msg) {
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

  if (visual_ == nullptr) {
    Ogre::RTShader::ShaderGenerator::initialize();

    Ogre::RTShader::ShaderGenerator* mShaderGenerator =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();

    mShaderGenerator->addSceneManager(context_->getSceneManager());

    // Grab the scheme render state.
    Ogre::RTShader::RenderState* schemRenderState =
        mShaderGenerator->getRenderState(
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    // Add per pixel lighting sub render state to the global scheme render
    // state. It will override the default FFP lighting sub render state.
    Ogre::RTShader::SubRenderState* perPixelLightModel =
        mShaderGenerator->createSubRenderState(
            Ogre::RTShader::PerPixelLighting::Type);
    schemRenderState->addTemplateSubRenderState(perPixelLightModel);

    visual_.reset(
        new VoxbloxMinimalMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMinimalMeshDisplay,
                       rviz::Display)
