#ifndef VOXBLOX_SKELETONS_SPARSE_GRAPH_PLANNER_H_
#define VOXBLOX_SKELETONS_SPARSE_GRAPH_PLANNER_H_

#include "voxblox/core/common.h"
#include "voxblox/skeletons/skeleton.h"
#include "voxblox/skeletons/neighbor_tools.h"
#include "voxblox/skeletons/nanoflann_interface.h"

namespace voxblox {

class SparseGraphPlanner {
 public:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<FloatingPoint, SkeletonVertexMapAdapter>,
      SkeletonVertexMapAdapter, 3> VertexGraphKdTree;

  SparseGraphPlanner();

  void setGraph(SparseSkeletonGraph* graph) {
    CHECK_NOTNULL(graph);
    graph_ = graph;
  }

  // Creates the kD trees. MUST be called before the first planning iteration.
  void setup();

  // Creates a path from the nearest vertex to the start position to the nearest
  // vertex to the end position. Off-graph planning is left as an exercise to
  // the reader (hint: use skeleton_planner).
  bool getPath(const Point& start_position, const Point& end_position,
               AlignedVector<Point>* coordinate_path) const;

  size_t getNClosestVertices(const Point& point, int num_vertices,
                             std::vector<int64_t>* vertex_inds) const;

  // Gets the path between vertex IDs.
  bool getPathBetweenVertices(int64_t start_vertex_id, int64_t end_vertex_id,
                              std::vector<int64_t>* vertex_path) const;

 private:
  int64_t popSmallestFromOpen(
      const std::map<int64_t, FloatingPoint>& f_score_map,
      std::set<int64_t>* open_set) const;

  void getSolutionPath(int64_t end_vertex_id,
                       const std::map<int64_t, int64_t>& parent_map,
                       std::vector<int64_t>* vertex_path) const;

  SparseSkeletonGraph* graph_;

  std::unique_ptr<VertexGraphKdTree> kd_tree_;
  std::unique_ptr<SkeletonVertexMapAdapter> kd_tree_adapter_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SPARSE_GRAPH_PLANNER_H_
