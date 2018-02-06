#ifndef VOXBLOX_SKELETONS_SKELETON_H_
#define VOXBLOX_SKELETONS_SKELETON_H_

#include "voxblox/core/common.h"

namespace voxblox {

struct SkeletonPoint {
  Point point;
  float distance;
  int num_basis_points;
  AlignedVector<Point> basis_directions;
};

class Skeleton {
 public:
  Skeleton();

  // Access to all the skeleton points.
  const AlignedVector<SkeletonPoint>& getSkeletonPoints() const {
    return points_;
  }

  AlignedVector<SkeletonPoint> getSkeletonPoints() { return points_; }

  // Converts the points to a pointcloud with no other information.
  void getPointcloud(Pointcloud* pointcloud) const;

  // Also get a vector for the distance information.
  void getPointcloudWithDistances(Pointcloud* pointcloud,
                                  std::vector<float>* distances) const;

 private:
  AlignedVector<SkeletonPoint> points_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_SKELETON_H_
