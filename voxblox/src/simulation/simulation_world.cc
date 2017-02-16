#include "voxblox/simulation/simulation_world.h"

namespace voxblox {

SimulationWorld::SimulationWorld() {}

void SimulationWorld::addObject(std::unique_ptr<Object> object) {
  // ??? Is this ok?
  objects_.push_back(std::move(object));
}

void SimulationWorld::addGroundLevel(FloatingPoint height) {}

void SimulationWorld::addWallBoundaries(FloatingPoint x_min,
                                        FloatingPoint x_max,
                                        FloatingPoint y_min,
                                        FloatingPoint y_max) {}

void SimulationWorld::getPointcloudFromViewpoint(
    const Eigen::Vector2i& camera_res, FloatingPoint fov_h_rad,
    FloatingPoint max_dist, Pointcloud* ptcloud) const {}

}  // namespace voxblox
