#include "voxblox/skeletons/skeleton.h"

namespace voxblox {

Skeleton::Skeleton() {}

void Skeleton::getPointcloud(Pointcloud* pointcloud) const {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  pointcloud->reserve(edges_.size());

  for (const SkeletonPoint& point : edges_) {
    pointcloud->push_back(point.point);
  }
}

void Skeleton::getPointcloudWithDistances(Pointcloud* pointcloud,
                                          std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(points_.size());
  distances->reserve(points_.size());

  for (const SkeletonPoint& point : points_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getEdgePointcloudWithDistances(
    Pointcloud* pointcloud, std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(edges_.size());
  distances->reserve(edges_.size());

  for (const SkeletonPoint& point : edges_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getVertexPointcloudWithDistances(
    Pointcloud* pointcloud, std::vector<float>* distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(vertices_.size());
  distances->reserve(vertices_.size());

  for (const SkeletonPoint& point : vertices_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

SparseSkeletonGraph::SparseSkeletonGraph()
    : next_vertex_id_(0), next_edge_id_(0) {}

int64_t SparseSkeletonGraph::addVertex(const SkeletonVertex& vertex) {
  int64_t vertex_id = next_vertex_id_++;
  // Returns an iterator...
  auto iter_pair = vertex_map_.emplace(std::make_pair(vertex_id, vertex));
  iter_pair.first->second.vertex_id = vertex_id;
  return vertex_id;
}

int64_t SparseSkeletonGraph::addEdge(const SkeletonEdge& edge) {
  int64_t edge_id = next_edge_id_++;
  // Returns an iterator...
  auto iter_pair = edge_map_.emplace(std::make_pair(edge_id, edge));
  iter_pair.first->second.edge_id = edge_id;

  // Now hook it up to the vertices.
  SkeletonVertex& start_vertex = vertex_map_[edge.start_vertex];
  SkeletonVertex& end_vertex = vertex_map_[edge.end_vertex];

  start_vertex.edge_list.push_back(edge_id);
  end_vertex.edge_list.push_back(edge_id);

  iter_pair.first->second.start_point = start_vertex.point;
  iter_pair.first->second.end_point = end_vertex.point;

  return edge_id;
}

bool SparseSkeletonGraph::hasVertex(int64_t id) const {
  if (vertex_map_.find(id) != vertex_map_.end()) {
    return true;
  }
  return false;
}

bool SparseSkeletonGraph::hasEdge(int64_t id) const {
  if (edge_map_.find(id) != edge_map_.end()) {
    return true;
  }
  return false;
}

const SkeletonVertex& SparseSkeletonGraph::getVertex(int64_t id) const {
  return vertex_map_.find(id)->second;
}

const SkeletonEdge& SparseSkeletonGraph::getEdge(int64_t id) const {
  return edge_map_.find(id)->second;
}

SkeletonVertex& SparseSkeletonGraph::getVertex(int64_t id) {
  return vertex_map_.find(id)->second;
}

SkeletonEdge& SparseSkeletonGraph::getEdge(int64_t id) {
  return edge_map_.find(id)->second;
}

void SparseSkeletonGraph::clear() {
  next_vertex_id_ = 0;
  next_edge_id_ = 0;

  vertex_map_.clear();
  edge_map_.clear();
}

void SparseSkeletonGraph::getAllVertexIds(
    std::vector<int64_t>* vertex_ids) const {
  vertex_ids->reserve(vertex_map_.size());
  for (const std::pair<int64_t, SkeletonVertex>& kv : vertex_map_) {
    vertex_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::getAllEdgeIds(std::vector<int64_t>* edge_ids) const {
  edge_ids->reserve(edge_map_.size());
  for (const std::pair<int64_t, SkeletonEdge>& kv : edge_map_) {
    edge_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::removeVertex(int64_t vertex_id) {
  // Find the vertex.
  std::map<int64_t, SkeletonVertex>::iterator iter =
      vertex_map_.find(vertex_id);
  if (iter == vertex_map_.end()) {
    return;
  }
  const SkeletonVertex& vertex = iter->second;

  // Remove all edges that are connected to it.
  for (int64_t edge_id : vertex.edge_list) {
    removeEdge(edge_id);
  }

  // Remove the vertex.
  vertex_map_.erase(iter);
}

void SparseSkeletonGraph::removeEdge(int64_t edge_id) {
  // Find the edge.
  std::map<int64_t, SkeletonEdge>::iterator iter = edge_map_.find(edge_id);
  if (iter == edge_map_.end()) {
    return;
  }
  const SkeletonEdge& edge = iter->second;

  // Remove this edge from both vertices.
  SkeletonVertex& vertex_1 = getVertex(edge.start_vertex);
  SkeletonVertex& vertex_2 = getVertex(edge.end_vertex);

  for (size_t i = 0; i < vertex_1.edge_list.size(); i++) {
    if (vertex_1.edge_list[i] == edge_id) {
      vertex_1.edge_list.erase(vertex_1.edge_list.begin() + i);
      break;
    }
  }
  for (size_t i = 0; i < vertex_2.edge_list.size(); i++) {
    if (vertex_2.edge_list[i] == edge_id) {
      vertex_2.edge_list.erase(vertex_2.edge_list.begin() + i);
      break;
    }
  }

  edge_map_.erase(iter);
}

bool SparseSkeletonGraph::areVerticesDirectlyConnected(
    int64_t vertex_id_1, int64_t vertex_id_2) const {
  const SkeletonVertex& vertex_1 = getVertex(vertex_id_1);
  // Iterate over all edges of vertex 1.
  for (int64_t edge_id : vertex_1.edge_list) {
    // Check if any connect to vertex 2.
    const SkeletonEdge& edge = getEdge(edge_id);
    if (edge.start_vertex == vertex_id_2 || edge.end_vertex == vertex_id_2) {
      return true;
    }
  }

  return false;
}
}  // namespace voxblox
