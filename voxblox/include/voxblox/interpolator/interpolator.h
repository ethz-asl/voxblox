#ifndef VOXBLOX_INTERPOLATOR_H_
#define VOXBLOX_INTERPOLATOR_H_

namespace voxblox {

class Interpolator {
 public:
  Interpolator(const Layer<TsdfVoxel>& tsdf_layer);

  bool getGradient(const Point& pos, Point* grad,
                   const bool interpolate = false) const;

  bool getDistance(const Point& pos, FloatingPoint* distance,
                   bool interpolate = false) const;

 private:
  static AnyIndex isNegitive(AnyIndex index);

  bool setIndexes(const Point& pos, BlockIndex* block_index,
                  InterpIndexes* voxel_indexes) const;

  void getWeightsVector(const Point& voxel_pos, const Point& pos,
                        InterpVector* weights_vector) const;

  bool getDistancesAndWeights(const BlockIndex& block_index,
                              const InterpIndexes& voxel_indexes,
                              const Point& pos, InterpVector* distances,
                              InterpVector* weights_vector) const;

  bool getInterpDistance(const Point& pos, FloatingPoint* distance) const;

  bool getNearestDistance(const Point& pos, FloatingPoint* distance) const;

  const Layer<TsdfVoxel>& tsdf_layer_;
};
}

#include "voxblox/interpolator/interpolator_inl.h"

#endif  // VOXBLOX_INTERPOLATOR_H_