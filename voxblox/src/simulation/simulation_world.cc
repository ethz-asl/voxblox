#include "voxblox/simulation/simulation_world.h"

namespace voxblox {

SimulationWorld::SimulationWorld()
    : min_bound_(-5.0, -5.0, -1.0), max_bound_(5.0, 5.0, 5.0) {}

void SimulationWorld::addObject(std::unique_ptr<Object> object) {
  objects_.push_back(std::move(object));
}

void SimulationWorld::addGroundLevel(FloatingPoint height) {
  objects_.emplace_back(
      new Plane(Point(0.0, 0.0, height), Point(0.0, 0.0, 1.0)));
}

void SimulationWorld::addPlaneBoundaries(FloatingPoint x_min,
                                         FloatingPoint x_max,
                                         FloatingPoint y_min,
                                         FloatingPoint y_max) {}

void SimulationWorld::clear() { objects_.clear(); }

void SimulationWorld::getPointcloudFromViewpoint(
    const Eigen::Vector2i& camera_res, FloatingPoint fov_h_rad,
    FloatingPoint max_dist, Pointcloud* ptcloud) const {}

}  // namespace voxblox
