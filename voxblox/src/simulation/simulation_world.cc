#include "voxblox/simulation/simulation_world.h"

namespace voxblox {

SimulationWorld::SimulationWorld()
    : min_bound_(-5.0, -5.0, -1.0), max_bound_(5.0, 5.0, 9.0) {}

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
                                         FloatingPoint y_max) {
  // X planes:
  objects_.emplace_back(
      new Plane(Point(x_min, 0.0, 0.0), Point(1.0, 0.0, 0.0)));
  objects_.emplace_back(
      new Plane(Point(x_max, 0.0, 0.0), Point(-1.0, 0.0, 0.0)));

  // Y planes:
  objects_.emplace_back(
      new Plane(Point(0.0, y_min, 0.0), Point(0.0, 1.0, 0.0)));
  objects_.emplace_back(
      new Plane(Point(0.0, y_max, 0.0), Point(0.0, -1.0, 0.0)));
}

void SimulationWorld::clear() { objects_.clear(); }

FloatingPoint SimulationWorld::getDistanceToPoint(
    const Point& coords, FloatingPoint max_dist) const {
  FloatingPoint min_dist = max_dist;
  for (size_t j = 0; j < objects_.size(); ++j) {
    FloatingPoint object_dist = objects_[j]->getDistanceToPoint(coords);
    if (object_dist < min_dist) {
      min_dist = object_dist;
    }
  }

  return min_dist;
}

void SimulationWorld::getPointcloudFromViewpoint(
    const Point& view_origin, const Point& view_direction,
    const Eigen::Vector2i& camera_res, FloatingPoint fov_h_rad,
    FloatingPoint max_dist, Pointcloud* ptcloud, Colors* colors) const {
  // Focal length based on fov.
  FloatingPoint focal_length = camera_res.x() / (2 * tan(fov_h_rad / 2.0));

  // Calculate transformation between nominal camera view direction and our
  // view direction. Nominal view is positive x direction.
  Point nominal_view_direction(1.0, 0.0, 0.0);
  Eigen::Quaternion<FloatingPoint> ray_rotation =
      Eigen::Quaternion<FloatingPoint>::FromTwoVectors(nominal_view_direction,
                                                       view_direction);

  // Now actually iterate over all pixels.
  for (int u = -camera_res.x() / 2; u < camera_res.x() / 2; ++u) {
    for (int v = -camera_res.y() / 2; v < camera_res.y() / 2; ++v) {
      Point ray_camera_direction =
          Point(1.0, u / focal_length, v / focal_length);
      Point ray_direction = ray_rotation * (ray_camera_direction).normalized();

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
