#ifndef KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_
#define KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_

#include <cmath>
#include <limits>

#include <glog/logging.h>

namespace kindr {
namespace minimal {

template <typename Scalar>
Transformation2DTemplate<Scalar>::Transformation2DTemplate() {
  setIdentity();
}

template <typename Scalar>
Transformation2DTemplate<Scalar>::Transformation2DTemplate(
    const Rotation r_A_B, const Position& A_t_A_B)
    : r_A_B_(r_A_B), A_t_A_B_(A_t_A_B) {}

template <typename Scalar>
Transformation2DTemplate<Scalar>::Transformation2DTemplate(
    const TransformationMatrix& T)
    : Transformation2DTemplate<Scalar>(
          Rotation2D().fromRotationMatrix(T.template topLeftCorner<2, 2>().eval()),
          T.template topRightCorner<2, 1>().eval()) {
  constexpr Scalar kEpsilon = std::numeric_limits<Scalar>::epsilon();
  CHECK_LE((T(2, 2) - static_cast<Scalar>(1.0)), kEpsilon);
  const Eigen::Matrix<Scalar, 2, 2> rotation_matrix =
      T.template topLeftCorner<2, 2>().eval();
  CHECK_NEAR(rotation_matrix.determinant(), static_cast<Scalar>(1.0), kEpsilon);
}

template <typename Scalar>
Transformation2DTemplate<Scalar>::~Transformation2DTemplate() {}

template <typename Scalar>
void Transformation2DTemplate<Scalar>::setIdentity() {
  r_A_B_ = Rotation2D(static_cast<Scalar>(0.0));
  A_t_A_B_.setZero();
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::Position&
Transformation2DTemplate<Scalar>::getPosition() {
  return A_t_A_B_;
}

template <typename Scalar>
const typename Transformation2DTemplate<Scalar>::Position&
Transformation2DTemplate<Scalar>::getPosition() const {
  return A_t_A_B_;
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::Rotation&
Transformation2DTemplate<Scalar>::getRotation() {
  return r_A_B_;
}

template <typename Scalar>
const typename Transformation2DTemplate<Scalar>::Rotation&
Transformation2DTemplate<Scalar>::getRotation() const {
  return r_A_B_;
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::RotationMatrix
Transformation2DTemplate<Scalar>::getRotationMatrix() const {
  return r_A_B_.toRotationMatrix();
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::TransformationMatrix
Transformation2DTemplate<Scalar>::getTransformationMatrix() const {
  TransformationMatrix transformation_matrix;
  transformation_matrix.template topLeftCorner<2, 2>() =
      r_A_B_.toRotationMatrix();
  transformation_matrix.template topRightCorner<2, 1>() = A_t_A_B_;
  transformation_matrix.template bottomRows<1>() =
      (Eigen::Matrix<Scalar, 1, 3>() << static_cast<Scalar>(0.0),
       static_cast<Scalar>(0.0), static_cast<Scalar>(1.0))
          .finished();
  return transformation_matrix;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Transformation2DTemplate<Scalar>::asVector() const {
  return (Eigen::Matrix<Scalar, 3, 1>() << r_A_B_.angle(), A_t_A_B_).finished();
}

template <typename Scalar>
Transformation2DTemplate<Scalar> Transformation2DTemplate<Scalar>::operator*(
    const Transformation2DTemplate<Scalar>& rhs) const {
  return Transformation2DTemplate<Scalar>(
      r_A_B_ * rhs.r_A_B_, A_t_A_B_ + r_A_B_ * rhs.A_t_A_B_);
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::Vector2
    Transformation2DTemplate<Scalar>::operator*(const Vector2& rhs) const {
  return transform(rhs);
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::Vector2
Transformation2DTemplate<Scalar>::transform(const Vector2& rhs) const {
  return r_A_B_ * rhs + A_t_A_B_;
}

template <typename Scalar>
typename Transformation2DTemplate<Scalar>::Matrix2X
Transformation2DTemplate<Scalar>::transformVectorized(
    const Matrix2X& rhs) const {
  return (r_A_B_.toRotationMatrix() * rhs).colwise() + A_t_A_B_;
}

template <typename Scalar>
Transformation2DTemplate<Scalar> Transformation2DTemplate<Scalar>::inverse()
    const {
  return Transformation2DTemplate<Scalar>(
      r_A_B_.inverse(), -(r_A_B_.inverse() * A_t_A_B_));
}

template <typename Scalar>
bool Transformation2DTemplate<Scalar>::operator==(
    const Transformation2DTemplate<Scalar>& rhs) const {
  return r_A_B_.angle() == rhs.r_A_B_.angle() && A_t_A_B_ == rhs.A_t_A_B_;
}

template <typename Scalar>
bool Transformation2DTemplate<Scalar>::operator!=(
    const Transformation2DTemplate<Scalar>& rhs) const {
  return !(*this == rhs);
}

template <typename Scalar>
std::ostream & operator<<(std::ostream & out,
                          const Transformation2DTemplate<Scalar>& rhs) {
  out << "[" <<rhs.getRotation().angle() << ", ["
      << rhs.getPosition().transpose() << "]]";
  return out;
}

template <typename Scalar>
template <typename ScalarAfterCast>
Transformation2DTemplate<ScalarAfterCast>
Transformation2DTemplate<Scalar>::cast() const {
  return Transformation2DTemplate<ScalarAfterCast>(
      getRotation().template cast<ScalarAfterCast>(),
      getPosition().template cast<ScalarAfterCast>());
}

}  // namespace minimal
}  // namespace kindr

#endif  // KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_
