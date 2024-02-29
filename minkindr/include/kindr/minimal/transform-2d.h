#ifndef KINDR_MINIMAL_TRANSFORM_2D_H_
#define KINDR_MINIMAL_TRANSFORM_2D_H_

#include <ostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kindr {
namespace minimal {

template <typename Scalar>
using Position2DTemplate = Eigen::Matrix<Scalar, 2, 1>;

template <typename Scalar>
using Rotation2DTemplate = Eigen::Rotation2D<Scalar>;

template <typename Scalar>
class Transformation2DTemplate {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

  using Rotation = Rotation2DTemplate<Scalar>;
  using Position = Position2DTemplate<Scalar>;
  using RotationMatrix = Eigen::Matrix<Scalar, 2, 2>;
  using TransformationMatrix = Eigen::Matrix<Scalar, 3, 3>;

  Transformation2DTemplate();

  Transformation2DTemplate(const Rotation r_A_B, const Position& A_t_A_B);

  explicit Transformation2DTemplate(const TransformationMatrix& T);

  ~Transformation2DTemplate();

  void setIdentity();

  // Non-const getter for the position vector.
  Position& getPosition();

  // Const getter for the position vector.
  const Position& getPosition() const;

  // Non-const getter for the rotation. Setting a new rotation angle can be done
  // as follows:
  //
  // Transformation2D T;
  // T.getRotation().angle() = new_angle;
  Rotation& getRotation();

  // Const getter for the rotation.
  const Rotation& getRotation() const;

  RotationMatrix getRotationMatrix() const;

  TransformationMatrix getTransformationMatrix() const;

  // Get the rotation angle and the position as a vector: [angle, x, y]
  Eigen::Matrix<Scalar, 3, 1> asVector() const;

  // Compose two transformations.
  Transformation2DTemplate<Scalar> operator*(
      const Transformation2DTemplate<Scalar>& rhs) const;

  // Transform a point.
  Vector2 operator*(const Vector2& rhs) const;

  // Transform a point.
  Vector2 transform(const Vector2& rhs) const;

  // Transform points.
  Matrix2X transformVectorized(const Matrix2X& rhs) const;

  // Returns a copy of the inverted transformation.
  Transformation2DTemplate<Scalar> inverse() const;

  // Check binary equality.
  bool operator==(const Transformation2DTemplate<Scalar>& rhs) const;

  // Check binary inequality.
  bool operator!=(const Transformation2DTemplate<Scalar>& rhs) const;

  // Cast scalar elements to another type.
  template <typename ScalarAfterCast>
  Transformation2DTemplate<ScalarAfterCast> cast() const;

 private:
  // The rotation that takes vectors from B to A.
  //
  // A_v = r_A_B * B_v;
  Rotation r_A_B_;

  // The vector from the origin of A to the origin of B, expressed in A.
  Position A_t_A_B_;
};

using Position2D = Position2DTemplate<double>;
using Rotation2D = Rotation2DTemplate<double>;
using Transformation2D = Transformation2DTemplate<double>;

template<typename Scalar>
std::ostream & operator<<(std::ostream & out,
    const Transformation2DTemplate<Scalar>& rhs);

}  // namespace minimal
}  // namespace kindr

#include "kindr/minimal/implementation/transform-2d-inl.h"

#endif  // KINDR_MINIMAL_TRANSFORM_2D_H_
