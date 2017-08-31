#ifndef VOXBLOX_UTILS_CAMERA_MODEL_H
#define VOXBLOX_UTILS_CAMERA_MODEL_H

#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

// Represents a plane in Hesse normal form (normal + distance) for faster
// distance calculations to points.
class Plane {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Plane() : normal_(Point::Identity()), distance_(0) {}
  virtual ~Plane() {}

  void setFromPoints(const Point& p1, const Point& p2, const Point& p3);
  void setFromDistanceNormal(const Point& normal, double distance);

  // I guess more correctly, is the point on the correct side of the plane.
  bool isPointInside(const Point& point) const;

  Point normal() const { return normal_; }
  double distance() const { return distance_; }

 private:
  Point normal_;
  double distance_;
};

class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel() : initialized_(false) {}
  virtual ~CameraModel() {}

  // Set up the camera model, intrinsics and extrinsics.
  void setIntrinsicsFromFocalLength(
      const Eigen::Matrix<FloatingPoint, 2, 1>& resolution, double focal_length,
      double min_distance, double max_distance);
  void setIntrinsicsFromFoV(double horizontal_fov, double vertical_fov,
                            double min_distance, double max_distance);
  void setExtrinsics(const Transformation& T_C_B);

  // Get and set the current poses for the camera (should be called after
  // the camera is properly set up).
  Transformation getCameraPose() const;
  Transformation getBodyPose() const;
  // Set camera pose actually computes the new bounding plane positions.
  void setCameraPose(const Transformation& cam_pose);
  void setBodyPose(const Transformation& body_pose);

  // Check whether a point belongs in the current view.
  void getAabb(Point* aabb_min, Point* aabb_max) const;
  bool isPointInView(const Point& point) const;

 private:
  void calculateBoundingPlanes();

  bool initialized_;

  Transformation T_C_B_;  // Extrinsic calibration to body.
  Transformation T_G_C_;  // Current pose of the camera.

  // The original vertices of the frustum, in the axis-aligned coordinate frame
  // (before rotation).
  AlignedVector<Point> untransformed_corners_;

  // The 6 bounding planes for the current camera pose, and their corresponding
  // AABB (Axis Aligned Bounding Box).
  AlignedVector<Plane> bounding_planes_;
  Point aabb_min_;
  Point aabb_max_;
};
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_CAMERA_MODEL_H
