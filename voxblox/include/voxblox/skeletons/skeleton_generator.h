#ifndef VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_
#define VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/skeletons/neighbor_tools.h"
#include "voxblox/skeletons/skeleton.h"
#include "voxblox/skeletons/skeleton_planner.h"
#include "voxblox/skeletons/voxel_template_matcher.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

// TODO(helenol): move as an integrator?
class SkeletonGenerator {
 public:
  SkeletonGenerator();
  SkeletonGenerator(Layer<EsdfVoxel>* esdf_layer);

  void setEsdfLayer(Layer<EsdfVoxel>* esdf_layer);

  void generateSkeleton();
  void generateSparseGraph();
  // Split non-straight edges on the sparse graph.
  void splitEdges();

  // Additional helper functions, in case generate by neighbor layers is set.
  void generateEdgesByLayerNeighbors();
  void generateVerticesByLayerNeighbors();

  // Clear the current points in the skeleton and re-construct it based on
  // the layer.
  void updateSkeletonFromLayer();

  // Pruning function by fitting template neighbors.
  size_t pruneDiagramEdges();

  // Prune the vertices by taking only the one with the largest distance in a
  // certain search radius.
  // Demotes vertices back to edges.
  void pruneDiagramVertices();

  // Repair the sparse graph after everything is said and done. Does some
  // dark magic to re-establish graph connectivity.
  void repairGraph();

  // Skeleton access.
  const Skeleton& getSkeleton() const { return skeleton_; }
  Skeleton& getSkeleton() { return skeleton_; }

  // Sparse graph access.
  const SparseSkeletonGraph& getSparseGraph() const { return graph_; }
  SparseSkeletonGraph& getSparseGraph() { return graph_; }

  float getMinSeparationAngle() const { return min_separation_angle_; }
  void setMinSeparationAngle(float min_separation_angle) {
    min_separation_angle_ = min_separation_angle;
  }

  bool getGenerateByLayerNeighbors() const {
    return generate_by_layer_neighbors_;
  }
  void setGenerateByLayerNeighbors(bool generate_by_layer_neighbors) {
    generate_by_layer_neighbors_ = generate_by_layer_neighbors;
  }

  FloatingPoint getMinGvdDistance() const { return min_gvd_distance_; }
  void setMinGvdDistance(FloatingPoint min_gvd_distance) {
    min_gvd_distance_ = min_gvd_distance;
  }

  int getNumNeighborsForEdge() const { return num_neighbors_for_edge_; }
  void setNumNeighborsForEdge(int num_neighbors_for_edge) {
    num_neighbors_for_edge_ = num_neighbors_for_edge;
  }

  // Follow an edge through the layer, aborting when either no more neighbors
  // exist or a vertex is found.
  bool followEdge(const BlockIndex& start_block_index,
                  const VoxelIndex& start_voxel_index,
                  const Eigen::Vector3i& direction_from_vertex,
                  int64_t* connected_vertex_id, float* min_distance,
                  float* max_distance);

  size_t mapNeighborIndexToBitsetIndex(size_t neighbor_index) const;

  // Determine whether a point is simple or not.
  bool isSimplePoint(const std::bitset<27>& neighbors) const;
  void octreeLabeling(int octant, int label, std::vector<int>* cube) const;

  // Determine whether a point is an endpoint (using our own amazing algorithm).
  bool isEndPoint(const std::bitset<27>& neighbors) const;

  // Utility functions for edge stuff.
  FloatingPoint getMaxEdgeDistanceFromStraightLine(
      const Point& start, const Point& end,
      AlignedVector<Point>* coordinate_path, size_t* max_index);
  FloatingPoint getMaxEdgeDistanceOnPath(
      const Point& start, const Point& end,
      const AlignedVector<Point>& coordinate_path, size_t* max_index);

  // Get the skeleton layer.
  Layer<SkeletonVoxel>* getSkeletonLayer() { return skeleton_layer_.get(); }

  // Set the skeleton layer! Takes ownership.
  void setSkeletonLayer(Layer<SkeletonVoxel>* skeleton_layer);

  int recursivelyLabel(int64_t vertex_id, int subgraph_id);
  void tryToFindEdgesInCoordinatePath(
      const AlignedVector<Point>& coordinate_path, int subgraph_id_start,
      int subgraph_id_end, std::vector<int64_t>* new_edge_ids);
void splitSpecificEdges(const std::vector<int64_t>& starting_edge_ids);


 private:
  float min_separation_angle_;
  int esdf_voxels_per_side_;

  // Whether generate vertices/edges by number of basis points (false,
  // default)
  // or number of neighbors on the discretized medial axis (true).
  bool generate_by_layer_neighbors_;
  int num_neighbors_for_edge_;

  // What minimum radius to prune vertices within.
  FloatingPoint vertex_pruning_radius_;

  // Minimum distance that the GVD is computed at. 0 by default.
  FloatingPoint min_gvd_distance_;

  // Template matchers.
  VoxelTemplateMatcher pruning_template_matcher_;
  VoxelTemplateMatcher corner_template_matcher_;

  // Neighbor tools, for finding nearest neighbors of things.
  NeighborTools<SkeletonVoxel> neighbor_tools_;
  SkeletonAStar skeleton_planner_;

  Skeleton skeleton_;

  Layer<EsdfVoxel>* esdf_layer_;
  // Owned by the generator! Since it's an intermediate by-product of
  // constructing the graph.
  std::unique_ptr<Layer<SkeletonVoxel>> skeleton_layer_;

  SparseSkeletonGraph graph_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_GENERATOR_H_
