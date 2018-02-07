#include "voxblox/skeletons/skeleton.h"

namespace voxblox {

Skeleton::Skeleton() {}

void Skeleton::getPointcloud(Pointcloud* pointcloud) const {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  pointcloud->reserve(points_.size());

  for (const SkeletonPoint& point : points_) {
    if (point.num_basis_points >= 3) {
      pointcloud->push_back(point.point);
    }
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
    if (point.num_basis_points >= 3) {
      pointcloud->push_back(point.point);
      distances->push_back(point.distance);
    }
  }
}

}  // namespace voxblox
