#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

#include <limits>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <voxblox/mesh/mesh_utils.h>

namespace voxblox_rviz_plugin {

unsigned int VoxbloxMeshVisual::instance_counter_ = 0;

VoxbloxMeshVisual::VoxbloxMeshVisual(Ogre::SceneManager* scene_manager,
                                     Ogre::SceneNode* parent_node,
                                     std::string name_space)
    : scene_manager_(scene_manager),
      name_space_(std::move(name_space)),
      is_enabled_(true) {
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;
}

VoxbloxMeshVisual::~VoxbloxMeshVisual() {
  // Destroy all the objects
  for (auto& ogre_object_pair : object_map_) {
    scene_manager_->destroyManualObject(ogre_object_pair.second);
  }
}

void VoxbloxMeshVisual::setPose(const Ogre::Vector3& position,
                                const Ogre::Quaternion& orientation) {
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);
}

void VoxbloxMeshVisual::setMessage(const voxblox_msgs::Mesh::ConstPtr& msg,
                                   uint8_t alpha) {
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                                    mesh_block.index[2]);

    size_t vertex_index = 0u;
    voxblox::Mesh mesh;
    mesh.vertices.reserve(mesh_block.x.size());
    mesh.indices.reserve(mesh_block.x.size());

    // translate vertex data from message to voxblox mesh
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      // Each vertex is given as its distance from the blocks origin in units of
      // (2*block_size), see mesh_vis.h for the slightly convoluted
      // justification of the 2.
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      const float mesh_x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(index[0])) *
          msg->block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(index[1])) *
          msg->block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(index[2])) *
          msg->block_edge_length;

      mesh.indices.push_back(vertex_index++);
      mesh.vertices.emplace_back(mesh_x, mesh_y, mesh_z);
    }

    // calculate normals
    mesh.normals.reserve(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
      const voxblox::Point dir0 = mesh.vertices[i] - mesh.vertices[i + 1];
      const voxblox::Point dir1 = mesh.vertices[i] - mesh.vertices[i + 2];
      const voxblox::Point normal = dir0.cross(dir1).normalized();

      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
    }

    // add color information
    mesh.colors.reserve(mesh.vertices.size());
    const bool has_color = mesh_block.x.size() == mesh_block.r.size();
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      voxblox::Color color;
      if (has_color) {
        color.r = mesh_block.r[i];
        color.g = mesh_block.g[i];
        color.b = mesh_block.b[i];

      } else {
        // reconstruct normals coloring
        color.r = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].x() * 0.5f + 0.5f);
        color.g = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].y() * 0.5f + 0.5f);
        color.b = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].z() * 0.5f + 0.5f);
      }
      color.a = alpha;
      mesh.colors.push_back(color);
    }

    // connect mesh
    voxblox::Mesh connected_mesh;
    voxblox::createConnectedMesh(mesh, &connected_mesh);

    // create ogre object
    Ogre::ManualObject* ogre_object;
    const voxblox::AnyIndexHashMapType<
        Ogre::ManualObject*>::type::const_iterator it = object_map_.find(index);
    if (it != object_map_.end()) {
      // delete empty mesh blocks
      if (mesh_block.x.size() == 0) {
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
                                std::to_string(instance_number_) +
                                std::string(" ") + name_space_;
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(index, ogre_object));
      if (!is_enabled_) {
        ogre_object->setVisible(false);
      }
      frame_node_->attachObject(ogre_object);
    }

    DCHECK(ogre_object != nullptr);

    ogre_object->estimateVertexCount(connected_mesh.vertices.size());
    ogre_object->estimateIndexCount(connected_mesh.indices.size());
    std::string material_name("VoxbloxMaterial");
    if (alpha < std::numeric_limits<uint8_t>::max()) {
      material_name = "VoxbloxMaterialTransparent";
    }
    ogre_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (size_t i = 0; i < connected_mesh.vertices.size(); ++i) {
      // note calling position changes what vertex the color and normal calls
      // point to
      ogre_object->position(connected_mesh.vertices[i].x(),
                            connected_mesh.vertices[i].y(),
                            connected_mesh.vertices[i].z());

      ogre_object->normal(connected_mesh.normals[i].x(),
                          connected_mesh.normals[i].y(),
                          connected_mesh.normals[i].z());
      constexpr float color_conv_factor =
          1.0f / std::numeric_limits<uint8_t>::max();
      ogre_object->colour(
          color_conv_factor * static_cast<float>(connected_mesh.colors[i].r),
          color_conv_factor * static_cast<float>(connected_mesh.colors[i].g),
          color_conv_factor * static_cast<float>(connected_mesh.colors[i].b),
          color_conv_factor * static_cast<float>(connected_mesh.colors[i].a));
    }

    // needed for anything other than flat rendering
    for (const voxblox::VertexIndex index : connected_mesh.indices) {
      ogre_object->index(index);
    }

    ogre_object->end();
  }
}

void VoxbloxMeshVisual::setEnabled(bool enabled) {
  if (enabled && !is_enabled_) {
    // new enable
    for (auto& manual_object : object_map_) {
      manual_object.second->setVisible(true);
    }
  } else if (!enabled && is_enabled_) {
    // new disable
    for (auto& manual_object : object_map_) {
      manual_object.second->setVisible(false);
    }
  }
  is_enabled_ = enabled;
}

}  // namespace voxblox_rviz_plugin
