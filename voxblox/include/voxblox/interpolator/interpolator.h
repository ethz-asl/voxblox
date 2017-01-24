#ifndef VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_
#define VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_

#include <memory>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

template <typename VoxelType>
class Interpolator {
 public:
  typedef std::shared_ptr<Interpolator> Ptr;

  explicit Interpolator(Layer<VoxelType>* layer);

  bool getGradient(const Point& pos, Point* grad,
                   const bool interpolate = false) const;

  bool getDistance(const Point& pos, FloatingPoint* distance,
                   bool interpolate = false) const;

  // This tries to use whatever information is available to interpolate the
  // distance and gradient -- if only one side is available, for instance,
  // this will still estimate a 1-sided gradient. Should give the same results
  // as getGradient() and getDistance() if all neighbors are filled.
  bool getAdaptiveDistanceAndGradient(const Point& pos, FloatingPoint* distance,
                                      Point* grad) const;

  // Without interpolation.
  bool getNearestDistanceAndWeight(const Point& pos, FloatingPoint* distance,
                                   float* weight) const;

 private:
  bool setIndexes(const Point& pos, BlockIndex* block_index,
                  InterpIndexes* voxel_indexes) const;

  // Q vector from http://spie.org/samples/PM159.pdf
  // Relates the interpolation distance of any arbitrary point inside a voxel
  // to the values of the voxel corners.
  void getQVector(const Point& voxel_pos, const Point& pos,
                  InterpVector* q_vector) const;

  bool getDistancesAndQVector(const BlockIndex& block_index,
                              const InterpIndexes& voxel_indexes,
                              const Point& pos, InterpVector* distances,
                              InterpVector* q_vector) const;

  bool getInterpDistance(const Point& pos, FloatingPoint* distance) const;

  bool getNearestDistance(const Point& pos, FloatingPoint* distance) const;

  // Allow this class to be templated on all kinds of voxels.
  FloatingPoint getVoxelDistance(const VoxelType& voxel) const;
  float getVoxelWeight(const VoxelType& voxel) const;
  // Returns true if the voxel should be used in interpolation/gradient
  // calculation. False otherwise.
  bool isVoxelValid(const VoxelType& voxel) const;

  Layer<VoxelType>* layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_

#include "voxblox/interpolator/interpolator_inl.h"
