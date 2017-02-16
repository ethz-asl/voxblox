#ifndef VOXBLOX_SIMULATION_OBJECTS_H_
#define VOXBLOX_SIMULATION_OBJECTS_H_

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

// Heavily inspired by @mfehr's OccupancyGridGenerator.
namespace voxblox {

// Base class for objects.
// Should this be full virtual?
class Object {
 public:
  // A wall is an infinite plane.
  enum Type { kSphere = 0, kCube, kWall };

  Object(const Point& center, Type type)
      : Object(center, type, Color::White()) {}
  Object(const Point& center, Type type, const Color& color)
      : center_(center), type_(type), color_(color) {}
  virtual ~Object() {}

  // .... here go useful functions ...
  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    return 0.0;
  }

 protected:
  Point center_;
  Type type_;
  Color color_;
};

class Sphere : public Object {
 public:
  Sphere(const Point& center, FloatingPoint radius)
      : Object(center, Type::kSphere), radius_(radius) {}
  Sphere(const Point& center, FloatingPoint radius, const Color& color)
      : Object(center, Type::kSphere, color), radius_(radius) {}

 protected:
  FloatingPoint radius_;
};

class Cube : public Object {};

class Wall : public Object {};

}  // namespace voxblox

#endif  // VOXBLOX_SIMULATION_OBJECTS_H_
