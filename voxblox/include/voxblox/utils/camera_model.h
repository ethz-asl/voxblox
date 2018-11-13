#ifndef VOXBLOX_UTILS_CAMERA_MODEL_H_
#define VOXBLOX_UTILS_CAMERA_MODEL_H_

#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

/**
 * Represents a plane in Hesse normal form (normal + distance) for faster
 * distance calculations to points.
 */
class Plane {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Plane() : normal_(Point::Identity()), distance_(0) {}
  virtual ~Plane() {}

  void setFromPoints(const Point& p1, const Point& p2, const Point& p3);
  void setFromDistanceNormal(const Point& normal, double distance);

  /// I guess more correctly, is the point on the correct side of the plane.
  bool isPointInside(const Point& point) const;

  Point normal() const { return normal_; }
  double distance() const { return distance_; }

 private:
  Point normal_;
  double distance_;
};

/**
 * Virtual camera model for use in simulating a systems view
 */
class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel() : initialized_(false) {}
  virtual ~CameraModel() {}

  /// Set up the camera model, intrinsics and extrinsics.
  void setIntrinsicsFromFocalLength(
      const Eigen::Matrix<FloatingPoint, 2, 1>& resolution, double focal_length,
      double min_distance, double max_distance);
  void setIntrinsicsFromFoV(double horizontal_fov, double vertical_fov,
                            double min_distance, double max_distance);
  void setExtrinsics(const Transformation& T_C_B);

  /**
   * Get and set the current poses for the camera (should be called after
   * the camera is properly set up).
   */
  Transformation getCameraPose() const;
  Transformation getBodyPose() const;
  /// Set camera pose actually computes the new bounding plane positions.
  void setCameraPose(const Transformation& cam_pose);
  void setBodyPose(const Transformation& body_pose);

  /// Check whether a point belongs in the current view.
  void getAabb(Point* aabb_min, Point* aabb_max) const;
  bool isPointInView(const Point& point) const;

  /**
   * Accessor functions for visualization (or other reasons).
   * Bounding planes are returned in the global coordinate frame.
   */
  const AlignedVector<Plane>& getBoundingPlanes() const {
    return bounding_planes_;
  }
  /**
   * Gives all the bounding lines of the planes in connecting order, expressed
   * in the global coordinate frame.
   */
  void getBoundingLines(AlignedVector<Point>* lines) const;

  /**
   * Get the 3 points definining the plane at the back (far end) of the camera
   * frustum. Expressed in global coordinates.
   */
  void getFarPlanePoints(AlignedVector<Point>* points) const;

 private:
  void calculateBoundingPlanes();

  bool initialized_;

  /// Extrinsic calibration to body.
  Transformation T_C_B_;
  /// Current pose of the camera.
  Transformation T_G_C_;

  /**
   * The original vertices of the frustum, in the axis-aligned coordinate frame
   * (before rotation).
   */
  AlignedVector<Point> corners_C_;

  /**
   * The 6 bounding planes for the current camera pose, and their corresponding
   * AABB (Axis Aligned Bounding Box). Expressed in global coordinate frame.
   */
  AlignedVector<Plane> bounding_planes_;
  Point aabb_min_;
  Point aabb_max_;
};
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_CAMERA_MODEL_H_
