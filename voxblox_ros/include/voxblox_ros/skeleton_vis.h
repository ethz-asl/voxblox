#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <voxblox/core/common.h>
#include <voxblox/skeletons/skeleton.h>

#include "voxblox_ros/conversions.h"

namespace voxblox {

inline void visualizeSkeletonGraph(
    const SparseSkeletonGraph& graph, const std::string& frame_id,
    visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  // Get a list of all vertices and visualize them as spheres.
  std::vector<int64_t> vertex_ids;
  graph.getAllVertexIds(&vertex_ids);

  visualization_msgs::Marker vertex_marker;
  vertex_marker.points.reserve(vertex_ids.size());

  vertex_marker.header.frame_id = frame_id;
  vertex_marker.ns = "vertices";
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.pose.orientation.w = 1.0;
  vertex_marker.scale.x = 0.2;
  vertex_marker.scale.y = vertex_marker.scale.x;
  vertex_marker.scale.z = vertex_marker.scale.x;
  vertex_marker.color.r = 1.0;
  vertex_marker.color.a = 1.0;

  for (int64_t vertex_id : vertex_ids) {
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(graph.getVertex(vertex_id).point.cast<double>(),
                        point_msg);
    vertex_marker.points.push_back(point_msg);
  }

  marker_array->markers.push_back(vertex_marker);

  // Get all edges and visualize as lines.
  std::vector<int64_t> edge_ids;
  graph.getAllEdgeIds(&edge_ids);

  visualization_msgs::Marker edge_marker;
  edge_marker.points.reserve(edge_ids.size() * 2);

  edge_marker.header.frame_id = frame_id;
  edge_marker.ns = "edges";
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.1;
  edge_marker.scale.y = edge_marker.scale.x;
  edge_marker.scale.z = edge_marker.scale.x;
  edge_marker.color.b = 1.0;
  edge_marker.color.a = 1.0;

  for (int64_t edge_id : edge_ids) {
    geometry_msgs::Point point_msg;
    const SkeletonEdge& edge = graph.getEdge(edge_id);
    tf::pointEigenToMsg(edge.start_point.cast<double>(), point_msg);
    edge_marker.points.push_back(point_msg);
    tf::pointEigenToMsg(edge.end_point.cast<double>(), point_msg);
    edge_marker.points.push_back(point_msg);
  }

  marker_array->markers.push_back(edge_marker);

  // Create 2 markers per vertex and edge - the point and line markers,
  // and the free-space path markers.
}

}  // namespace voxblox
