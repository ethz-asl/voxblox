
#ifndef VOXBLOX_FAST_SIMULATION_SIMULATION_WORLD_H_
#define VOXBLOX_FAST_SIMULATION_SIMULATION_WORLD_H_

#include <vector>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/simulation/objects.h"

namespace voxblox_fast {

class SimulationWorld {
 public:
  SimulationWorld();
  virtual ~SimulationWorld() {}

  // === Creating an environment ===
  void addObject(std::unique_ptr<Object> object);

  // Convenience functions for setting up bounded areas.
  // Ground level can also be used for ceiling. ;)
  void addGroundLevel(FloatingPoint height);
  // Add 4 walls (infinite planes) bounding the space. In case this is not the
  // desired behavior, can use addObject to add walls manually one by one.
  // If infinite walls are undesirable, then use cubes.
  void addPlaneBoundaries(FloatingPoint x_min, FloatingPoint x_max,
                          FloatingPoint y_min, FloatingPoint y_max);

  // Deletes all objects!
  void clear();

  // === Generating synthetic data from environment ===
  // Generates a synthetic view
  // Assumes square pixels for ease... Takes in FoV in radians.
  void getPointcloudFromViewpoint(const Point& view_origin,
                                  const Point& view_direction,
                                  const Eigen::Vector2i& camera_res,
                                  FloatingPoint fov_h_rad,
                                  FloatingPoint max_dist, Pointcloud* ptcloud,
                                  Colors* colors) const;

  // === Computing ground truth SDFs ===
  //// ??? How to do this for both ESDF and TSDF and whatever?
  template <typename VoxelType>
  void generateSdfFromWorld(FloatingPoint max_dist,
                            Layer<VoxelType>* layer) const;

  FloatingPoint getDistanceToPoint(const Point& coords,
                                   FloatingPoint max_dist) const;

  Point getMinBound() const { return min_bound_; }
  Point getMaxBound() const { return max_bound_; }

 private:
  template <typename VoxelType>
  void setVoxel(FloatingPoint dist, const Color& color, VoxelType* voxel) const;

  // Vector storing pointers to all the objects in this world.
  std::vector<std::unique_ptr<Object> > objects_;

  // World boundaries... Can be changed arbitrarily, just sets ground truth
  // generation and visualization bounds, accurate only up to block size.
  Point min_bound_;
  Point max_bound_;
};

}  // namespace voxblox

#endif  // VOXBLOX_FAST_SIMULATION_SIMULATION_WORLD_H_

#include "voxblox_fast/simulation/simulation_world_inl.h"
