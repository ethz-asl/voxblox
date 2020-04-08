#ifndef VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_
#define VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_

#include <memory>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

/**
 * Interpolates voxels to give distances and gradients
 */
template <typename VoxelType>
class Interpolator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Interpolator> Ptr;

  explicit Interpolator(const Layer<VoxelType>* layer);

  bool getGradient(const Point& pos, Point* grad,
                   const bool interpolate = false) const;

  bool getDistance(const Point& pos, FloatingPoint* distance,
                   bool interpolate = false) const;

  bool getWeight(const Point& pos, FloatingPoint* weight,
                 bool interpolate = false) const;

  bool getVoxel(const Point& pos, VoxelType* voxel,
                bool interpolate = false) const;

  /**
   * This tries to use whatever information is available to interpolate the
   * distance and gradient -- if only one side is available, for instance,
   * this will still estimate a 1-sided gradient. Should give the same results
   * as getGradient() and getDistance() if all neighbors are filled.
   */
  bool getAdaptiveDistanceAndGradient(const Point& pos, FloatingPoint* distance,
                                      Point* grad) const;

  /// Without interpolation.
  bool getNearestDistanceAndWeight(const Point& pos, FloatingPoint* distance,
                                   float* weight) const;

  bool setIndexes(const Point& pos, BlockIndex* block_index,
                  InterpIndexes* voxel_indexes) const;

  bool getVoxelsAndQVector(const Point& pos, const VoxelType** voxels,
                           InterpVector* q_vector) const;

 private:
  /**
   * Q vector from http://spie.org/samples/PM159.pdf
   * Relates the interpolation distance of any arbitrary point inside a voxel
   * to the values of the voxel corners.
   */
  void getQVector(const Point& voxel_pos, const Point& pos,
                  const FloatingPoint voxel_size_inv,
                  InterpVector* q_vector) const;

  bool getVoxelsAndQVector(const BlockIndex& block_index,
                           const InterpIndexes& voxel_indexes, const Point& pos,
                           const VoxelType** voxels,
                           InterpVector* q_vector) const;

  bool getInterpDistance(const Point& pos, FloatingPoint* distance) const;

  bool getNearestDistance(const Point& pos, FloatingPoint* distance) const;

  bool getInterpWeight(const Point& pos, FloatingPoint* weight) const;

  bool getNearestWeight(const Point& pos, FloatingPoint* weight) const;

  bool getInterpVoxel(const Point& pos, VoxelType* voxel) const;

  bool getNearestVoxel(const Point& pos, VoxelType* voxel) const;

  static float getVoxelSdf(const VoxelType& voxel);
  static float getVoxelWeight(const VoxelType& voxel);

  static uint8_t getRed(const VoxelType& voxel);
  static uint8_t getBlue(const VoxelType& voxel);
  static uint8_t getGreen(const VoxelType& voxel);
  static uint8_t getAlpha(const VoxelType& voxel);

  template <typename TGetter>
  static FloatingPoint interpMember(const InterpVector& q_vector,
                                    const VoxelType** voxels,
                                    TGetter (*getter)(const VoxelType&));

  static VoxelType interpVoxel(const InterpVector& q_vector,
                               const VoxelType** voxels);

  const Layer<VoxelType>* layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTERPOLATOR_INTERPOLATOR_H_

#include "voxblox/interpolator/interpolator_inl.h"
