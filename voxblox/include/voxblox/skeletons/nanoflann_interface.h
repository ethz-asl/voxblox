#ifndef VOXBLOX_SKELETONS_NANOFLANN_INTERFACE_H_
#define VOXBLOX_SKELETONS_NANOFLANN_INTERFACE_H_

#include "voxblox/nanoflann/nanoflann.h"

#include "voxblox/skeletons/skeleton.h"

namespace voxblox {

// Adaptor is British spelling of Adapter...
class SkeletonPointVectorAdapter {
 public:
  SkeletonPointVectorAdapter(const AlignedVector<SkeletonPoint>& points)
      : points_(points) {}

  inline const AlignedVector<SkeletonPoint>& derived() const { return points_; }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return points_.size(); }

  // Returns the dim'th component of the idx'th point in the class.
  inline FloatingPoint kdtree_get_pt(const size_t idx, int dim) const {
    return points_[idx].point(dim);
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const {
    return false;
  }

 private:
  const AlignedVector<SkeletonPoint>& points_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SKELETONS_NANOFLANN_INTERFACE_H_
