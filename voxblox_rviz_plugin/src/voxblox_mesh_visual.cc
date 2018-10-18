#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

unsigned int VoxbloxMeshVisual::instance_counter_ = 0;

VoxbloxMeshVisual::VoxbloxMeshVisual(Ogre::SceneManager* scene_manager,
                                     Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;
}

VoxbloxMeshVisual::~VoxbloxMeshVisual() {
  // Destroy all the objects
  for (std::pair<const voxblox::BlockIndex, Ogre::ManualObject*> ogre_object :
       object_map_) {
    scene_manager_->destroyManualObject(ogre_object.second);
  }
}

void VoxbloxMeshVisual::setMessage(const voxblox_msgs::Mesh::ConstPtr& msg) {
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                                    mesh_block.index[2]);

    Ogre::ManualObject* ogre_object;
    const voxblox::AnyIndexHashMapType<
        Ogre::ManualObject*>::type::const_iterator it = object_map_.find(index);
    if (it != object_map_.end()) {
      // delete empty mesh blocks
      if (mesh_block.triangles.size() == 0) {
        scene_manager_->destroyManualObject(it->second);
        object_map_.erase(it);
        continue;
      }

      ogre_object = it->second;
      ogre_object->clear();
    } else {
      std::string object_name = std::to_string(index.x()) + std::string(" ") +
                                std::to_string(index.y()) + std::string(" ") +
                                std::to_string(index.z()) + std::string(" ") +
                                std::to_string(instance_number_);
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(index, ogre_object));

      frame_node_->attachObject(ogre_object);
    }

    DCHECK(ogre_object != nullptr);

    ogre_object->estimateVertexCount(3 * mesh_block.triangles.size());
    ogre_object->begin("BaseWhiteNoLighting",
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (const voxblox_msgs::Triangle& triangle : mesh_block.triangles) {
      for (size_t i = 0; i < 3; ++i) {
        // sanity checks
        if (std::isfinite(triangle.x[i]) && std::isfinite(triangle.y[i]) &&
            std::isfinite(triangle.z[i])) {
          ogre_object->position(triangle.x[i], triangle.y[i], triangle.z[i]);

          constexpr float color_conv_factor = 1.0f / 255.0f;
          ogre_object->colour(
              color_conv_factor * static_cast<float>(triangle.r[i]),
              color_conv_factor * static_cast<float>(triangle.g[i]),
              color_conv_factor * static_cast<float>(triangle.b[i]),
              color_conv_factor * static_cast<float>(triangle.a[i]));
        }
      }
    }

    ogre_object->end();
  }
}

void VoxbloxMeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void VoxbloxMeshVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

unsigned int VoxbloxMinimalMeshVisual::instance_counter_ = 0;

VoxbloxMinimalMeshVisual::VoxbloxMinimalMeshVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;
}

VoxbloxMinimalMeshVisual::~VoxbloxMinimalMeshVisual() {
  // Destroy all the objects
  for (std::pair<const voxblox::BlockIndex, Ogre::ManualObject*> ogre_object :
       object_map_) {
    scene_manager_->destroyManualObject(ogre_object.second);
  }
}

void VoxbloxMinimalMeshVisual::setMessage(
    const voxblox_msgs::MinimalMesh::ConstPtr& msg) {
  for (const voxblox_msgs::MinimalMeshBlock& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                                    mesh_block.index[2]);

    Ogre::ManualObject* ogre_object;
    const voxblox::AnyIndexHashMapType<
        Ogre::ManualObject*>::type::const_iterator it = object_map_.find(index);
    if (it != object_map_.end()) {
      // delete empty mesh blocks
      if (mesh_block.triangles.size() == 0) {
        scene_manager_->destroyManualObject(it->second);
        object_map_.erase(it);
        continue;
      }

      ogre_object = it->second;
      ogre_object->clear();
    } else {
      std::string object_name = std::to_string(index.x()) + std::string(" ") +
                                std::to_string(index.y()) + std::string(" ") +
                                std::to_string(index.z()) + std::string(" ") +
                                std::to_string(instance_number_);
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(index, ogre_object));

      frame_node_->attachObject(ogre_object);
    }

    DCHECK(ogre_object != nullptr);

    ogre_object->estimateVertexCount(3 * mesh_block.triangles.size());
    ogre_object->begin("BaseWhiteNoLighting",
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (const voxblox_msgs::MinimalTriangle& triangle : mesh_block.triangles) {
      Ogre::Vector3 points[3];

      for (size_t i = 0; i < 3; ++i) {
        constexpr float point_conv_factor = 2.0f / 65535.0f;
        const float mesh_x =
            (static_cast<float>(triangle.x[i]) * point_conv_factor +
             static_cast<float>(index[0])) *
            msg->block_edge_length;
        const float mesh_y =
            (static_cast<float>(triangle.y[i]) * point_conv_factor +
             static_cast<float>(index[1])) *
            msg->block_edge_length;
        const float mesh_z =
            (static_cast<float>(triangle.z[i]) * point_conv_factor +
             static_cast<float>(index[2])) *
            msg->block_edge_length;

        points[i] = Ogre::Vector3(mesh_x, mesh_y, mesh_z);
      }

      // color by normals
      const Ogre::Vector3 dir0 = points[0] - points[1];
      const Ogre::Vector3 dir1 = points[0] - points[2];
      const Ogre::Vector3 normal = dir0.crossProduct(dir1).normalisedCopy();

      Ogre::ColourValue color;
      color.r = normal.x * 0.5f + 0.5f;
      color.g = normal.y * 0.5f + 0.5f;
      color.b = normal.z * 0.5f + 0.5f;
      color.a = 1.0f;

      // add triangle to ogre
      ogre_object->position(points[0]);
      ogre_object->colour(color);
      ogre_object->position(points[1]);
      ogre_object->colour(color);
      ogre_object->position(points[2]);
      ogre_object->colour(color);
    }

    ogre_object->end();
  }
}

void VoxbloxMinimalMeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void VoxbloxMinimalMeshVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin
