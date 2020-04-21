#include "voxblox/utils/camera_model.h"

namespace voxblox {

void Plane::setFromPoints(const Point& p1, const Point& p2, const Point& p3) {
  Point p1p2 = p2 - p1;
  Point p1p3 = p3 - p1;

  Point cross = p1p2.cross(p1p3);
  normal_ = cross.normalized();
  distance_ = normal_.dot(p1);
}

void Plane::setFromDistanceNormal(const Point& normal, double distance) {
  normal_ = normal;
  distance_ = distance;
}

bool Plane::isPointInside(const Point& point) const {
  VLOG(5) << "Plane: normal: " << normal_.transpose()
          << " distance: " << distance_ << " point: " << point.transpose();
  VLOG(5) << "Distance: " << point.dot(normal_) + distance_;
  if (point.dot(normal_) >= distance_) {
    return true;
  }
  return false;
}

// Set up the camera model, intrinsics and extrinsics.
void CameraModel::setIntrinsicsFromFocalLength(
    const Eigen::Matrix<FloatingPoint, 2, 1>& resolution, double focal_length,
    double min_distance, double max_distance) {
  // Figure out FOV from the given data...
  double horizontal_fov = 2 * std::atan(resolution.x() / (2 * focal_length));
  double vertical_fov = 2 * std::atan(resolution.y() / (2 * focal_length));

  setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance,
                       max_distance);
}

void CameraModel::setIntrinsicsFromFoV(double horizontal_fov,
                                       double vertical_fov, double min_distance,
                                       double max_distance) {
  // Given this information, create 6 bounding planes, assuming the camera is
  // pointing with in the positive X direction.
  // We create the planes by calculating all the corner points and store them
  // as corners_C_.
  // The planes are computed once a pose is set.
  corners_C_.clear();
  corners_C_.reserve(8);

  // First compute the near plane.
  double tan_half_horizontal_fov = std::tan(horizontal_fov / 2.0);
  double tan_half_vertical_fov = std::tan(vertical_fov / 2.0);
  corners_C_.emplace_back(Point(min_distance,
                                min_distance * tan_half_horizontal_fov,
                                min_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(min_distance,
                                min_distance * tan_half_horizontal_fov,
                                -min_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(min_distance,
                                -min_distance * tan_half_horizontal_fov,
                                -min_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(min_distance,
                                -min_distance * tan_half_horizontal_fov,
                                min_distance * tan_half_vertical_fov));

  // Then the far plane is more or less the same.
  corners_C_.emplace_back(Point(max_distance,
                                max_distance * tan_half_horizontal_fov,
                                max_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(max_distance,
                                max_distance * tan_half_horizontal_fov,
                                -max_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(max_distance,
                                -max_distance * tan_half_horizontal_fov,
                                -max_distance * tan_half_vertical_fov));
  corners_C_.emplace_back(Point(max_distance,
                                -max_distance * tan_half_horizontal_fov,
                                max_distance * tan_half_vertical_fov));

  initialized_ = true;
}

void CameraModel::setExtrinsics(const Transformation& T_C_B) { T_C_B_ = T_C_B; }

// Get and set the current poses for the camera (should be called after
// the camera is properly set up).
Transformation CameraModel::getCameraPose() const { return T_G_C_; }

Transformation CameraModel::getBodyPose() const { return T_G_C_ * T_C_B_; }

void CameraModel::setCameraPose(const Transformation& cam_pose) {
  T_G_C_ = cam_pose;

  calculateBoundingPlanes();
}

void CameraModel::setBodyPose(const Transformation& body_pose) {
  setCameraPose(body_pose * T_C_B_.inverse());
}

void CameraModel::calculateBoundingPlanes() {
  if (!initialized_) {
    return;
  }

  CHECK_EQ(corners_C_.size(), 8u);
  if (bounding_planes_.empty()) {
    bounding_planes_.resize(6);
  }

  AlignedVector<Point> corners_G;
  corners_G.resize(corners_C_.size());

  // Transform all the points.
  for (size_t i = 0; i < corners_C_.size(); ++i) {
    corners_G[i] = T_G_C_ * corners_C_[i];
  }

  // Near plane.
  bounding_planes_[0].setFromPoints(corners_G[0], corners_G[2], corners_G[1]);
  VLOG(5) << "Near plane: Normal: " << bounding_planes_[0].normal().transpose()
          << " distance: " << bounding_planes_[0].distance();
  // Far plane.
  bounding_planes_[1].setFromPoints(corners_G[4], corners_G[5], corners_G[6]);
  VLOG(5) << "Far plane: Normal: " << bounding_planes_[1].normal().transpose()
          << " distance: " << bounding_planes_[1].distance();

  // Left.
  bounding_planes_[2].setFromPoints(corners_G[3], corners_G[6], corners_G[2]);
  VLOG(5) << "Left plane: Normal: " << bounding_planes_[2].normal().transpose()
          << " distance: " << bounding_planes_[2].distance();

  // Right.
  bounding_planes_[3].setFromPoints(corners_G[0], corners_G[5], corners_G[4]);
  VLOG(5) << "Right plane: Normal: " << bounding_planes_[3].normal().transpose()
          << " distance: " << bounding_planes_[3].distance();

  // Top.
  bounding_planes_[4].setFromPoints(corners_G[3], corners_G[4], corners_G[7]);
  VLOG(5) << "Top plane: Normal: " << bounding_planes_[4].normal().transpose()
          << " distance: " << bounding_planes_[4].distance();

  // Bottom.
  bounding_planes_[5].setFromPoints(corners_G[2], corners_G[6], corners_G[5]);
  VLOG(5) << "Bottom plane: Normal: "
          << bounding_planes_[5].normal().transpose()
          << " distance: " << bounding_planes_[5].distance();

  // Calculate AABB.
  aabb_min_.setConstant(std::numeric_limits<double>::max());
  aabb_max_.setConstant(std::numeric_limits<double>::lowest());

  for (int i = 0; i < 3; i++) {
    for (size_t j = 0; j < corners_G.size(); j++) {
      aabb_min_(i) = std::min(aabb_min_(i), corners_G[j](i));
      aabb_max_(i) = std::max(aabb_max_(i), corners_G[j](i));
    }
  }

  VLOG(5) << "AABB min:\n"
          << aabb_min_.transpose() << "\nAABB max:\n"
          << aabb_max_.transpose();
}

void CameraModel::getAabb(Point* aabb_min, Point* aabb_max) const {
  *aabb_min = aabb_min_;
  *aabb_max = aabb_max_;
}

bool CameraModel::isPointInView(const Point& point) const {
  // Skip the AABB check, assume already been done.
  for (size_t i = 0; i < bounding_planes_.size(); i++) {
    if (!bounding_planes_[i].isPointInside(point)) {
      return false;
    }
  }
  return true;
}

void CameraModel::getBoundingLines(AlignedVector<Point>* lines) const {
  CHECK_NOTNULL(lines);
  lines->clear();
  lines->reserve(24);

  // Transform the points again... This is just for visualization so we can
  // waste a bit more time here.
  AlignedVector<Point> corners_G;
  // Untransformed corners are in the camera coordinate frame (corners_C_).
  corners_G.resize(corners_C_.size());

  // Transform all the points.
  for (size_t i = 0; i < corners_C_.size(); ++i) {
    corners_G[i] = T_G_C_ * corners_C_[i];
  }

  // All pairs of lines.
  lines->push_back(corners_G[0]);
  lines->push_back(corners_G[1]);

  lines->push_back(corners_G[1]);
  lines->push_back(corners_G[2]);

  lines->push_back(corners_G[2]);
  lines->push_back(corners_G[3]);

  lines->push_back(corners_G[3]);
  lines->push_back(corners_G[0]);

  lines->push_back(corners_G[4]);
  lines->push_back(corners_G[5]);

  lines->push_back(corners_G[5]);
  lines->push_back(corners_G[6]);

  lines->push_back(corners_G[6]);
  lines->push_back(corners_G[7]);

  lines->push_back(corners_G[7]);
  lines->push_back(corners_G[4]);

  lines->push_back(corners_G[0]);
  lines->push_back(corners_G[4]);

  lines->push_back(corners_G[1]);
  lines->push_back(corners_G[5]);

  lines->push_back(corners_G[3]);
  lines->push_back(corners_G[7]);

  lines->push_back(corners_G[2]);
  lines->push_back(corners_G[6]);
}

void CameraModel::getFarPlanePoints(AlignedVector<Point>* points) const {
  CHECK_NOTNULL(points);
  points->clear();
  points->reserve(3);

  points->push_back(T_G_C_ * corners_C_[4]);
  points->push_back(T_G_C_ * corners_C_[5]);
  points->push_back(T_G_C_ * corners_C_[6]);
}

}  // namespace voxblox
