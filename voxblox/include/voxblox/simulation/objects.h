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
  enum Type { kSphere = 0, kCube, kPlane };

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

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    FloatingPoint distance = (center_ - point).norm() - radius_;
    return distance;
  }

 protected:
  FloatingPoint radius_;
};



class Cube : public Object {};


// Requires normal being passed in to ALREADY BE NORMALIZED!!!!
class Plane : public Object {
 public:
  Plane(const Point& center, const Point& normal)
      : Object(center, Type::kPlane), normal_(normal) {}
  Plane(const Point& center, const Point& normal, const Color& color)
      : Object(center, Type::kPlane, color), normal_(normal) {}

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    // Compute the 'd' in ax + by + cz + d = 0:
    // This is actually the scalar product I guess.
    FloatingPoint d = -normal_.dot(center_);
    FloatingPoint p = d/normal_.norm();

    FloatingPoint distance = normal_.dot(point) + p;
    return distance;
  }

 protected:
  Point normal_;
};


}  // namespace voxblox

#endif  // VOXBLOX_SIMULATION_OBJECTS_H_
