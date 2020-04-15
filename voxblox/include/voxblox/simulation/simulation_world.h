#ifndef VOXBLOX_SIMULATION_SIMULATION_WORLD_H_
#define VOXBLOX_SIMULATION_SIMULATION_WORLD_H_

#include <list>
#include <memory>
#include <random>
#include <vector>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/simulation/objects.h"

namespace voxblox {

class SimulationWorld {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimulationWorld();
  virtual ~SimulationWorld() {}

  /// === Creating an environment ===
  void addObject(std::unique_ptr<Object> object);

  /**
   * Convenience functions for setting up bounded areas.
   * Ground level can also be used for ceiling. ;)
   */
  void addGroundLevel(FloatingPoint height);

  /**
   * Add 4 walls (infinite planes) bounding the space. In case this is not the
   * desired behavior, can use addObject to add walls manually one by one.
   * If infinite walls are undesirable, then use cubes.
   */
  void addPlaneBoundaries(FloatingPoint x_min, FloatingPoint x_max,
                          FloatingPoint y_min, FloatingPoint y_max);

  /// Deletes all objects!
  void clear();

  /** === Generating synthetic data from environment ===
   * Generates a synthetic view
   * Assumes square pixels for ease... Takes in FoV in radians.
   */
  void getPointcloudFromViewpoint(const Point& view_origin,
                                  const Point& view_direction,
                                  const Eigen::Vector2i& camera_res,
                                  FloatingPoint fov_h_rad,
                                  FloatingPoint max_dist, Pointcloud* ptcloud,
                                  Colors* colors) const;
  void getPointcloudFromTransform(const Transformation& pose,
                                  const Eigen::Vector2i& camera_res,
                                  FloatingPoint fov_h_rad,
                                  FloatingPoint max_dist, Pointcloud* ptcloud,
                                  Colors* colors) const;

  /**
   * Same thing as getPointcloudFromViewpoint, but also adds a noise in the
   * *distance* of the measurement, given by noise_sigma (Gaussian noise). No
   * noise in the bearing.
   */
  void getNoisyPointcloudFromViewpoint(const Point& view_origin,
                                       const Point& view_direction,
                                       const Eigen::Vector2i& camera_res,
                                       FloatingPoint fov_h_rad,
                                       FloatingPoint max_dist,
                                       FloatingPoint noise_sigma,
                                       Pointcloud* ptcloud, Colors* colors);
  void getNoisyPointcloudFromTransform(const Transformation& pose,
                                       const Eigen::Vector2i& camera_res,
                                       FloatingPoint fov_h_rad,
                                       FloatingPoint max_dist,
                                       FloatingPoint noise_sigma,
                                       Pointcloud* ptcloud, Colors* colors);

  // === Computing ground truth SDFs ===
  template <typename VoxelType>
  void generateSdfFromWorld(FloatingPoint max_dist,
                            Layer<VoxelType>* layer) const;

  FloatingPoint getDistanceToPoint(const Point& coords,
                                   FloatingPoint max_dist) const;

  /// Set and get the map generation and display bounds.
  void setBounds(const Point& min_bound, const Point& max_bound) {
    min_bound_ = min_bound;
    max_bound_ = max_bound;
  }

  Point getMinBound() const { return min_bound_; }
  Point getMaxBound() const { return max_bound_; }

 protected:
  template <typename VoxelType>
  void setVoxel(FloatingPoint dist, const Color& color, VoxelType* voxel) const;

  FloatingPoint getNoise(FloatingPoint noise_sigma);

  /// List storing pointers to all the objects in this world.
  std::list<std::unique_ptr<Object> > objects_;

  // World boundaries... Can be changed arbitrarily, just sets ground truth
  // generation and visualization bounds, accurate only up to block size.
  Point min_bound_;
  Point max_bound_;

  /// For producing noise. Sets a fixed seed (0).
  std::default_random_engine generator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SIMULATION_SIMULATION_WORLD_H_

#include "voxblox/simulation/simulation_world_inl.h"
