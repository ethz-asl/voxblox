#include "voxblox/mesh/mesh.h"

namespace voxblox {

void Mesh::concatenateMesh(const Mesh& mesh) {
  // Concatenating verticies
  if (mesh.hasVertices()) {
    this->vertices.insert(this->vertices.end(), mesh.vertices.begin(),
                          mesh.vertices.end());
  }
  // Concatenating indicies
  if (mesh.hasIndices()) {
    VertexIndexList indicies_for_concatentation = mesh.indices;
    size_t current_mesh_size = this->indices.size();
    if (current_mesh_size > 0) {
      std::for_each(indicies_for_concatentation.begin(),
                    indicies_for_concatentation.end(),
                    [current_mesh_size](VertexIndex& vertex_index) {
                      vertex_index = vertex_index + current_mesh_size;
                    });
    }
    this->indices.insert(this->indices.end(),
                         indicies_for_concatentation.begin(),
                         indicies_for_concatentation.end());
  }
  // Concatenating verticies
  if (mesh.hasNormals()) {
    this->normals.insert(this->normals.end(), mesh.normals.begin(),
                         mesh.normals.end());
  }
  // Concatenating verticies
  if (mesh.hasColors()) {
    this->colors.insert(this->colors.end(), mesh.colors.begin(),
                        mesh.colors.end());
  }
}

}  // namespace voxblox
