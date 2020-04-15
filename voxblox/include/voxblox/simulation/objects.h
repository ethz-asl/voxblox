#ifndef VOXBLOX_SIMULATION_OBJECTS_H_
#define VOXBLOX_SIMULATION_OBJECTS_H_

#include <algorithm>
#include <iostream>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

// Heavily inspired by @mfehr's OccupancyGridGenerator.
namespace voxblox {

// Should this be full virtual?
/**
 * Base class for simulator objects. Each object allows an exact ground-truth
 * sdf to be created for it.
 */
class Object {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // A wall is an infinite plane.
  enum Type { kSphere = 0, kCube, kPlane, kCylinder };

  Object(const Point& center, Type type)
      : Object(center, type, Color::White()) {}
  Object(const Point& center, Type type, const Color& color)
      : center_(center), type_(type), color_(color) {}
  virtual ~Object() {}

  /// Map-building accessors.
  virtual FloatingPoint getDistanceToPoint(const Point& point) const = 0;

  Color getColor() const { return color_; }
  Type getType() const { return type_; }

  /// Raycasting accessors.
  virtual bool getRayIntersection(const Point& ray_origin,
                                  const Point& ray_direction,
                                  FloatingPoint max_dist,
                                  Point* intersect_point,
                                  FloatingPoint* intersect_dist) const = 0;

 protected:
  Point center_;
  Type type_;
  Color color_;
};

class Sphere : public Object {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sphere(const Point& center, FloatingPoint radius)
      : Object(center, Type::kSphere), radius_(radius) {}
  Sphere(const Point& center, FloatingPoint radius, const Color& color)
      : Object(center, Type::kSphere, color), radius_(radius) {}

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    FloatingPoint distance = (center_ - point).norm() - radius_;
    return distance;
  }

  virtual bool getRayIntersection(const Point& ray_origin,
                                  const Point& ray_direction,
                                  FloatingPoint max_dist,
                                  Point* intersect_point,
                                  FloatingPoint* intersect_dist) const {
    // From https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // x = o + dl is the ray equation
    // r = sphere radius, c = sphere center
    FloatingPoint under_square_root =
        pow(ray_direction.dot(ray_origin - center_), 2) -
        (ray_origin - center_).squaredNorm() + pow(radius_, 2);

    // No real roots = no intersection.
    if (under_square_root < 0.0) {
      return false;
    }

    FloatingPoint d =
        -(ray_direction.dot(ray_origin - center_)) - sqrt(under_square_root);

    // Intersection behind the origin.
    if (d < 0.0) {
      return false;
    }
    // Intersection greater than max dist, so no intersection in the sensor
    // range.
    if (d > max_dist) {
      return false;
    }

    *intersect_point = ray_origin + d * ray_direction;
    *intersect_dist = d;
    return true;
  }

 protected:
  FloatingPoint radius_;
};

class Cube : public Object {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cube(const Point& center, const Point& size)
      : Object(center, Type::kCube), size_(size) {}
  Cube(const Point& center, const Point& size, const Color& color)
      : Object(center, Type::kCube, color), size_(size) {}

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    // Solution from http://stackoverflow.com/questions/5254838/
    // calculating-distance-between-a-point-and-a-rectangular-box-nearest-point

    Point distance_vector = Point::Zero();
    distance_vector.x() =
        std::max(std::max(center_.x() - size_.x() / 2.0 - point.x(), 0.0),
                 point.x() - center_.x() - size_.x() / 2.0);
    distance_vector.y() =
        std::max(std::max(center_.y() - size_.y() / 2.0 - point.y(), 0.0),
                 point.y() - center_.y() - size_.y() / 2.0);
    distance_vector.z() =
        std::max(std::max(center_.z() - size_.z() / 2.0 - point.z(), 0.0),
                 point.z() - center_.z() - size_.z() / 2.0);

    FloatingPoint distance = distance_vector.norm();

    // Basically 0... Means it's inside!
    if (distance < kEpsilon) {
      distance_vector.x() = std::max(center_.x() - size_.x() / 2.0 - point.x(),
                                     point.x() - center_.x() - size_.x() / 2.0);
      distance_vector.y() = std::max(center_.y() - size_.y() / 2.0 - point.y(),
                                     point.y() - center_.y() - size_.y() / 2.0);
      distance_vector.z() = std::max(center_.z() - size_.z() / 2.0 - point.z(),
                                     point.z() - center_.z() - size_.z() / 2.0);
      distance = distance_vector.maxCoeff();
    }

    return distance;
  }

  virtual bool getRayIntersection(const Point& ray_origin,
                                  const Point& ray_direction,
                                  FloatingPoint max_dist,
                                  Point* intersect_point,
                                  FloatingPoint* intersect_dist) const {
    // Adapted from https://www.scratchapixel.com/lessons/3d-basic-rendering/
    // minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    // Compute min and max limits in 3D.

    // Precalculate signs and inverse directions.
    Point inv_dir(1.0 / ray_direction.x(), 1.0 / ray_direction.y(),
                  1.0 / ray_direction.z());
    Eigen::Vector3i ray_sign(inv_dir.x() < 0.0, inv_dir.y() < 0.0,
                             inv_dir.z() < 0.0);

    Point bounds[2];
    bounds[0] = center_ - size_ / 2.0;
    bounds[1] = center_ + size_ / 2.0;

    FloatingPoint tmin =
        (bounds[ray_sign.x()].x() - ray_origin.x()) * inv_dir.x();
    FloatingPoint tmax =
        (bounds[1 - ray_sign.x()].x() - ray_origin.x()) * inv_dir.x();
    FloatingPoint tymin =
        (bounds[ray_sign.y()].y() - ray_origin.y()) * inv_dir.y();
    FloatingPoint tymax =
        (bounds[1 - ray_sign.y()].y() - ray_origin.y()) * inv_dir.y();

    if ((tmin > tymax) || (tymin > tmax)) return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    FloatingPoint tzmin =
        (bounds[ray_sign.z()].z() - ray_origin.z()) * inv_dir.z();
    FloatingPoint tzmax =
        (bounds[1 - ray_sign.z()].z() - ray_origin.z()) * inv_dir.z();

    if ((tmin > tzmax) || (tzmin > tmax)) return false;
    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    FloatingPoint t = tmin;
    if (t < 0.0) {
      t = tmax;
      if (t < 0.0) {
        return false;
      }
    }

    if (t > max_dist) {
      return false;
    }

    *intersect_dist = t;
    *intersect_point = ray_origin + ray_direction * t;

    return true;
  }

 protected:
  Point size_;
};

/// Requires normal being passed in to ALREADY BE NORMALIZED!!!!
class PlaneObject : public Object {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneObject(const Point& center, const Point& normal)
      : Object(center, Type::kPlane), normal_(normal) {}
  PlaneObject(const Point& center, const Point& normal, const Color& color)
      : Object(center, Type::kPlane, color), normal_(normal) {
    CHECK_NEAR(normal.norm(), 1.0, 1e-3);
  }

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    // Compute the 'd' in ax + by + cz + d = 0:
    // This is actually the scalar product I guess.
    FloatingPoint d = -normal_.dot(center_);
    FloatingPoint p = d / normal_.norm();

    FloatingPoint distance = normal_.dot(point) + p;
    return distance;
  }

  virtual bool getRayIntersection(const Point& ray_origin,
                                  const Point& ray_direction,
                                  FloatingPoint max_dist,
                                  Point* intersect_point,
                                  FloatingPoint* intersect_dist) const {
    // From https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
    // Following notation of sphere more...
    // x = o + dl is the ray equation
    // n = normal, c = plane 'origin'
    FloatingPoint denominator = ray_direction.dot(normal_);
    if (std::abs(denominator) < kEpsilon) {
      // Lines are parallel, no intersection.
      return false;
    }
    FloatingPoint d = (center_ - ray_origin).dot(normal_) / denominator;
    if (d < 0.0) {
      return false;
    }
    if (d > max_dist) {
      return false;
    }
    *intersect_point = ray_origin + d * ray_direction;
    *intersect_dist = d;
    return true;
  }

 protected:
  Point normal_;
};

class Cylinder : public Object {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cylinder(const Point& center, FloatingPoint radius, FloatingPoint height)
      : Object(center, Type::kCylinder), radius_(radius), height_(height) {}
  Cylinder(const Point& center, FloatingPoint radius, FloatingPoint height,
           const Color& color)
      : Object(center, Type::kCylinder, color),
        radius_(radius),
        height_(height) {}

  virtual FloatingPoint getDistanceToPoint(const Point& point) const {
    // From: https://math.stackexchange.com/questions/2064745/
    // 3 cases, depending on z of point.
    // First case: in plane with the cylinder. This also takes care of inside
    // case.
    FloatingPoint distance = 0.0;

    FloatingPoint min_z_limit = center_.z() - height_ / 2.0;
    FloatingPoint max_z_limit = center_.z() + height_ / 2.0;
    if (point.z() >= min_z_limit && point.z() <= max_z_limit) {
      distance = (point.head<2>() - center_.head<2>()).norm() - radius_;
    } else if (point.z() > max_z_limit) {
      // Case 2: above the cylinder.
      distance = std::sqrt(
          std::max((point.head<2>() - center_.head<2>()).squaredNorm() -
                       radius_ * radius_,
                   static_cast<FloatingPoint>(0.0)) +
          (point.z() - max_z_limit) * (point.z() - max_z_limit));
    } else {
      // Case 3: below cylinder.
      distance = std::sqrt(
          std::max((point.head<2>() - center_.head<2>()).squaredNorm() -
                       radius_ * radius_,
                   static_cast<FloatingPoint>(0.0)) +
          (point.z() - min_z_limit) * (point.z() - min_z_limit));
    }
    return distance;
  }

  virtual bool getRayIntersection(const Point& ray_origin,
                                  const Point& ray_direction,
                                  FloatingPoint max_dist,
                                  Point* intersect_point,
                                  FloatingPoint* intersect_dist) const {
    // From http://woo4.me/wootracer/cylinder-intersection/
    // and http://www.cl.cam.ac.uk/teaching/1999/AGraphHCI/SMAG/node2.html
    // Define ray as P = E + tD, where E is ray_origin and D is ray_direction.
    // We define our cylinder as centered in the xy coordinate system, so
    // E in this case is actually ray_origin - center_.
    Point vector_E = ray_origin - center_;
    Point vector_D = ray_direction;  // Axis aligned.

    FloatingPoint a = vector_D.x() * vector_D.x() + vector_D.y() * vector_D.y();
    FloatingPoint b =
        2 * vector_E.x() * vector_D.x() + 2 * vector_E.y() * vector_D.y();
    FloatingPoint c = vector_E.x() * vector_E.x() +
                      vector_E.y() * vector_E.y() - radius_ * radius_;

    // t = (-b +- sqrt(b^2 - 4ac))/2a
    // t only has solutions if b^2 - 4ac >= 0
    FloatingPoint t1 = -1.0;
    FloatingPoint t2 = -1.0;

    // Make sure we don't divide by 0.
    if (std::abs(a) < kEpsilon) {
      return false;
    }

    FloatingPoint under_square_root = b * b - 4 * a * c;
    if (under_square_root < 0) {
      return false;
    }
    if (under_square_root <= kEpsilon) {
      t1 = -b / (2 * a);
      // Just keep t2 at invalid default value.
    } else {
      // 2 ts.
      t1 = (-b + std::sqrt(under_square_root)) / (2 * a);
      t2 = (-b - std::sqrt(under_square_root)) / (2 * a);
    }

    // Great, now we got some ts, time to figure out whether we hit the cylinder
    // or the endcaps.
    FloatingPoint t = max_dist;

    FloatingPoint z1 = vector_E.z() + t1 * vector_D.z();
    FloatingPoint z2 = vector_E.z() + t2 * vector_D.z();
    bool t1_valid = t1 >= 0.0 && z1 >= -height_ / 2.0 && z1 <= height_ / 2.0;
    bool t2_valid = t2 >= 0.0 && z2 >= -height_ / 2.0 && z2 <= height_ / 2.0;

    // Get the endcaps and their validity.
    // Check end-cap intersections now... :(
    FloatingPoint t3, t4;
    bool t3_valid = false, t4_valid = false;

    // Make sure we don't divide by 0.
    if (std::abs(vector_D.z()) > kEpsilon) {
      // t3 is the bottom end-cap, t4 is the top.
      t3 = (-height_ / 2.0 - vector_E.z()) / vector_D.z();
      t4 = (height_ / 2.0 - vector_E.z()) / vector_D.z();

      Point q3 = vector_E + t3 * vector_D;
      Point q4 = vector_E + t4 * vector_D;

      t3_valid = t3 >= 0.0 && q3.head<2>().norm() < radius_;
      t4_valid = t4 >= 0.0 && q4.head<2>().norm() < radius_;
    }

    if (!(t1_valid || t2_valid || t3_valid || t4_valid)) {
      return false;
    }
    if (t1_valid) {
      t = std::min(t, t1);
    }
    if (t2_valid) {
      t = std::min(t, t2);
    }
    if (t3_valid) {
      t = std::min(t, t3);
    }
    if (t4_valid) {
      t = std::min(t, t4);
    }

    // Intersection greater than max dist, so no intersection in the sensor
    // range.
    if (t >= max_dist) {
      return false;
    }

    // Back to normal coordinates now.
    *intersect_point = ray_origin + t * ray_direction;
    *intersect_dist = t;
    return true;
  }

 protected:
  FloatingPoint radius_;
  FloatingPoint height_;
};

}  // namespace voxblox

#endif  // VOXBLOX_SIMULATION_OBJECTS_H_
