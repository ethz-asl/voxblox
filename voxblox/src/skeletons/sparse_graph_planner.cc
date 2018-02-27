#include "voxblox/skeletons/sparse_graph_planner.h"

#include <map>
#include <set>

namespace voxblox {

SparseGraphPlanner::SparseGraphPlanner() {}

void SparseGraphPlanner::setup() {
  CHECK_NOTNULL(graph_);

  // Build the kD Tree of the vertices at the current moment.
  // Create the adapter.

  // construct a kd-tree index:
  const int kDim = 3;
  const int kMaxLeaf = 10;

  kd_tree_adapter_.reset(new SkeletonVertexMapAdapter(graph_->getVertexMap()));

  kd_tree_.reset(new VertexGraphKdTree(
      kDim, *kd_tree_adapter_,
      nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf)));

  kd_tree_->buildIndex();
}

bool SparseGraphPlanner::getPath(const Point& start_position,
                                 const Point& end_position,
                                 AlignedVector<Point>* coordinate_path) const {
  // Easiest solution: start with 1-NN for both start and end. Let's see if
  // this works.
  std::vector<int64_t> start_vertex_inds, end_vertex_inds;

  getNClosestVertices(start_position, 10, &start_vertex_inds);
  int64_t start_vertex_id = start_vertex_inds[0];
  getNClosestVertices(end_position, 10, &end_vertex_inds);
  int64_t end_vertex_id = end_vertex_inds[0];

  // Pick the best start and end.
  for (int64_t vertex_id : start_vertex_inds) {
    const SkeletonVertex& vertex = graph_->getVertex(vertex_id);
    if (vertex.edge_list.size() > 1) {
      LOG(INFO) << "Start point: " << vertex_id
                << " Position: " << vertex.point.transpose();
      start_vertex_id = vertex_id;
      break;
    }
  }
  for (int64_t vertex_id : end_vertex_inds) {
    const SkeletonVertex& vertex = graph_->getVertex(vertex_id);
    if (vertex.edge_list.size() > 1) {
      end_vertex_id = vertex_id;
      LOG(INFO) << "End point: " << vertex_id
                << " Position: " << vertex.point.transpose();
      break;
    }
  }

  std::vector<int64_t> vertex_path;
  if (!getPathBetweenVertices(start_vertex_id, end_vertex_id, &vertex_path)) {
    return false;
  }

  coordinate_path->clear();
  coordinate_path->reserve(vertex_path.size());
  for (int64_t vertex_id : vertex_path) {
    coordinate_path->push_back(graph_->getVertex(vertex_id).point);
  }

  return true;
}

bool SparseGraphPlanner::getPathBetweenVertices(
    int64_t start_vertex_id, int64_t end_vertex_id,
    std::vector<int64_t>* vertex_path) const {
  std::map<int64_t, FloatingPoint> f_score_map;
  std::map<int64_t, FloatingPoint> g_score_map;
  std::map<int64_t, int64_t> parent_map;

  std::set<int64_t> open_set;
  std::set<int64_t> closed_set;

  int64_t current_vertex_id = start_vertex_id;

  const SkeletonVertex& end_vertex = graph_->getVertex(end_vertex_id);
  const SkeletonVertex& this_vertex = graph_->getVertex(current_vertex_id);

  f_score_map[current_vertex_id] =
      (end_vertex.point - this_vertex.point).norm();
  g_score_map[current_vertex_id] = 0.0;

  open_set.insert(current_vertex_id);

  while (!open_set.empty()) {
    // Find the smallest f-value in the open set.
    current_vertex_id = popSmallestFromOpen(f_score_map, &open_set);

    const SkeletonVertex& vertex = graph_->getVertex(current_vertex_id);
    LOG(INFO) << " Closed set size: " << closed_set.size()
              << " Open set size: " << open_set.size()
              << " Current id: " << current_vertex_id
              << " Current position: " << vertex.point.transpose();
    // Check if this is already the goal...
    if (current_vertex_id == end_vertex_id) {
      getSolutionPath(end_vertex_id, parent_map, vertex_path);
      return true;
    }

    closed_set.insert(current_vertex_id);

    for (int64_t edge_id : vertex.edge_list) {
      const SkeletonEdge& edge = graph_->getEdge(edge_id);
      int64_t neighbor_vertex_id = -1;
      if (edge.start_vertex == current_vertex_id) {
        neighbor_vertex_id = edge.end_vertex;
      } else {
        neighbor_vertex_id = edge.start_vertex;
      }

      if (closed_set.count(neighbor_vertex_id) > 0) {
        // Already checked this guy as well.
        continue;
      }
      if (open_set.count(neighbor_vertex_id) == 0) {
        open_set.insert(neighbor_vertex_id);
      }

      const SkeletonVertex& neighbor_vertex =
          graph_->getVertex(neighbor_vertex_id);

      FloatingPoint tentative_g_score =
          g_score_map[current_vertex_id] +
          (neighbor_vertex.point - vertex.point).norm();
      if (g_score_map.count(neighbor_vertex_id) == 0 ||
          g_score_map[neighbor_vertex_id] < tentative_g_score) {
        g_score_map[neighbor_vertex_id] = tentative_g_score;
        f_score_map[neighbor_vertex_id] =
            tentative_g_score +
            (end_vertex.point - neighbor_vertex.point).norm();
        parent_map[neighbor_vertex_id] = current_vertex_id;
      }
    }
  }
  return false;
}

int64_t SparseGraphPlanner::popSmallestFromOpen(
    const std::map<int64_t, FloatingPoint>& f_score_map,
    std::set<int64_t>* open_set) const {
  FloatingPoint min_distance = std::numeric_limits<FloatingPoint>::max();
  std::set<int64_t>::const_iterator min_iter = open_set->cbegin();

  for (std::set<int64_t>::const_iterator iter = open_set->cbegin();
       iter != open_set->cend(); ++iter) {
    FloatingPoint distance = f_score_map.at(*iter);
    if (distance < min_distance) {
      min_distance = distance;
      min_iter = iter;
    }
  }

  int64_t return_val = *min_iter;
  open_set->erase(min_iter);
  return return_val;
}

void SparseGraphPlanner::getSolutionPath(
    int64_t end_vertex_id, const std::map<int64_t, int64_t>& parent_map,
    std::vector<int64_t>* vertex_path) const {
  CHECK_NOTNULL(vertex_path);
  vertex_path->clear();
  vertex_path->push_back(end_vertex_id);
  int64_t vertex_id = end_vertex_id;
  while (parent_map.count(vertex_id) > 0) {
    vertex_id = parent_map.at(vertex_id);
    vertex_path->push_back(vertex_id);
  }
  std::reverse(vertex_path->begin(), vertex_path->end());
}

size_t SparseGraphPlanner::getNClosestVertices(
    const Point& point, int num_vertices,
    std::vector<int64_t>* vertex_inds) const {
  CHECK(kd_tree_);
  CHECK(kd_tree_adapter_);
  CHECK_NOTNULL(graph_);
  CHECK_NOTNULL(vertex_inds);
  vertex_inds->clear();
  vertex_inds->reserve(num_vertices);

  std::vector<size_t> ret_index(num_vertices);
  std::vector<FloatingPoint> out_dist_sqr(num_vertices);

  nanoflann::SearchParams params;  // Defaults are fine.
  size_t num_results = kd_tree_->knnSearch(point.data(), num_vertices,
                                           &ret_index[0], &out_dist_sqr[0]);

  for (size_t map_index : ret_index) {
    auto iter = graph_->getVertexMap().begin();
    for (size_t i = 0; i < map_index; ++i) {
      iter++;
    }
    vertex_inds->push_back(iter->first);
  }
  return num_results;
}

}  // namespace voxblox
