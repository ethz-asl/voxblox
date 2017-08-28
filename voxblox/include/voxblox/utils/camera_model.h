#ifndef VOXBLOX_UTILS_CAMERA_MODEL_H
#define VOXBLOX_UTILS_CAMERA_MODEL_H

#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformationTemplate<double> Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<double> Rotation;

// Represents a plane in Hesse normal form (normal + distance) for faster
// distance calculations to points.
class Plane {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Plane() : normal_(Eigen::Vector3d::Identity()), distance_(0) {}
  virtual ~Plane() {}

  void setFromPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                     const Eigen::Vector3d& p3);
  void setFromDistanceNormal(const Eigen::Vector3d& normal, double distance);

  // I guess more correctly, is the point on the correct side of the plane.
  bool isPointInside(const Eigen::Vector3d& point) const;

  Eigen::Vector3d normal() const { return normal_; }
  double distance() const { return distance_; }

 private:
  Eigen::Vector3d normal_;
  double distance_;
};

class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel() : initialized_(false) {}
  virtual ~CameraModel() {}

  // Set up the camera model, intrinsics and extrinsics.
  void setIntrinsicsFromFocalLength(const Eigen::Vector2d& resolution,
                                    double focal_length, double min_distance,
                                    double max_distance);
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
  void getAabb(Eigen::Vector3d* aabb_min, Eigen::Vector3d* aabb_max) const;
  bool isPointInView(const Eigen::Vector3d& point) const;

 private:
  void calculateBoundingPlanes();

  bool initialized_;

  Transformation T_C_B_;  // Extrinsic calibration to body.
  Transformation T_G_C_;  // Current pose of the camera.

  // The original vertices of the frustum, in the axis-aligned coordinate frame
  // (before rotation).
  AlignedVector<Eigen::Vector3d> untransformed_corners_;

  // The 6 bounding planes for the current camera pose, and their corresponding
  // AABB (Axis Aligned Bounding Box).
  AlignedVector<Plane> bounding_planes_;
  Eigen::Vector3d aabb_min_;
  Eigen::Vector3d aabb_max_;
};
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_CAMERA_MODEL_H
