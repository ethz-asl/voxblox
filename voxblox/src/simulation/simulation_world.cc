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
    const Point& view_origin, const Point& view_direction,
    const Eigen::Vector2i& camera_res, FloatingPoint fov_h_rad,
    FloatingPoint max_dist, Pointcloud* ptcloud, Colors* colors) const {
  // First, figure out, for instance, how big each ray increment is.
  FloatingPoint fov_increment = camera_res.x() / fov_h_rad;
  FloatingPoint fov_v_rad = fov_h_rad * camera_res.y() / camera_res.x();

  // Now actually iterate over all pixels by fov_h_rad increments.
  for (FloatingPoint u_angle = -fov_h_rad / 2.0; u_angle <= fov_h_rad / 2.0;
       u_angle += fov_increment) {
    for (FloatingPoint v_angle = -fov_v_rad / 2.0; v_angle <= fov_v_rad / 2.0;
         v_angle += fov_increment) {
      // Direction for this ray is...
      Eigen::AngleAxis<FloatingPoint> u_rot(u_angle, Point::UnitZ());
      Eigen::AngleAxis<FloatingPoint> v_rot(v_angle, Point::UnitY());

      Point ray_direction = v_rot * u_rot * view_direction;
      // Cast this ray into every object.
      bool ray_valid = false;
      FloatingPoint ray_dist = max_dist;
      Point ray_intersect = Point::Zero();
      Color ray_color;
      for (size_t i = 0; i < objects_.size(); ++i) {
        Point object_intersect;
        FloatingPoint object_dist;
        bool intersects = objects_[i]->getRayIntersection(
            view_origin, ray_direction, max_dist, &object_intersect,
            &object_dist);
        if (intersects) {
          if (!ray_valid || object_dist < ray_dist) {
            ray_valid = true;
            ray_dist = object_dist;
            ray_intersect = object_intersect;
            ray_color = objects_[i]->getColor();
          }
        }
      }
      if (ray_valid) {
        ptcloud->push_back(ray_intersect);
        colors->push_back(ray_color);
      }
    }
  }
}

}  // namespace voxblox
