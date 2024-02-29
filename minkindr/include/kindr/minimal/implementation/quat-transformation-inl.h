// Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
#define KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
#include <kindr/minimal/quat-transformation.h>

#include <glog/logging.h>

namespace kindr {
namespace minimal {

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate() {
  setIdentity();
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
    const RotationQuaternionTemplate<Scalar>& q_A_B, const Position& A_t_A_B) :
    q_A_B_(q_A_B),
    A_t_A_B_(A_t_A_B) {

}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
    const typename Rotation::Implementation& q_A_B,
    const Position& A_t_A_B) :
        q_A_B_(q_A_B),
        A_t_A_B_(A_t_A_B) {

}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
    const Position& A_t_A_B, const RotationQuaternionTemplate<Scalar>& q_A_B) :
    q_A_B_(q_A_B),
    A_t_A_B_(A_t_A_B) {

}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
    const Position& A_t_A_B, const typename Rotation::Implementation& q_A_B) :
        q_A_B_(q_A_B),
        A_t_A_B_(A_t_A_B) {

}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
    const TransformationMatrix& T) :
    q_A_B_(T.template topLeftCorner<3,3>().eval()),
    A_t_A_B_(T.template topRightCorner<3,1>().eval()) {
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
     const QuatTransformationTemplate<Scalar>::Vector6& x_t_r) :
  q_A_B_(x_t_r.template tail<3>().eval()),
  A_t_A_B_(x_t_r.template head<3>().eval()) {
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>::~QuatTransformationTemplate() {

}

template<typename Scalar>
void QuatTransformationTemplate<Scalar>::setIdentity() {
  q_A_B_.setIdentity();
  A_t_A_B_.setZero();
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Position&
QuatTransformationTemplate<Scalar>::getPosition() {
  return A_t_A_B_;
}

template<typename Scalar>
const typename QuatTransformationTemplate<Scalar>::Position&
QuatTransformationTemplate<Scalar>::getPosition() const {
  return A_t_A_B_;
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Rotation&
QuatTransformationTemplate<Scalar>::getRotation() {
  return q_A_B_;
}

template<typename Scalar>
const typename QuatTransformationTemplate<Scalar>::Rotation&
QuatTransformationTemplate<Scalar>::getRotation() const {
  return q_A_B_;
}

template <typename Scalar>
const Eigen::Quaternion<Scalar>&
QuatTransformationTemplate<Scalar>::getEigenQuaternion() const {
  return getRotation().toImplementation();
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::TransformationMatrix
QuatTransformationTemplate<Scalar>::getTransformationMatrix() const {
  TransformationMatrix transformation_matrix;
  transformation_matrix.setIdentity();
  transformation_matrix.template topLeftCorner<3,3>() =
      q_A_B_.getRotationMatrix();
  transformation_matrix.template topRightCorner<3,1>() = A_t_A_B_;
  return transformation_matrix;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 7, 1>
QuatTransformationTemplate<Scalar>::asVector() const {
  return (Eigen::Matrix<Scalar, 7, 1>() <<
      q_A_B_.w(), q_A_B_.x(), q_A_B_.y(), q_A_B_.z(), A_t_A_B_).finished();
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::RotationMatrix
QuatTransformationTemplate<Scalar>::getRotationMatrix() const {
  return q_A_B_.getRotationMatrix();
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>
QuatTransformationTemplate<Scalar>::operator*(
    const QuatTransformationTemplate<Scalar>& rhs) const {
  return QuatTransformationTemplate<Scalar>(q_A_B_ * rhs.q_A_B_, A_t_A_B_ +
                                            q_A_B_.rotate(rhs.A_t_A_B_));
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector3
QuatTransformationTemplate<Scalar>::transform(
    const typename QuatTransformationTemplate<Scalar>::Vector3& rhs) const {
  return q_A_B_.rotate(rhs) + A_t_A_B_;
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Matrix3X
QuatTransformationTemplate<Scalar>::transformVectorized(
    const typename QuatTransformationTemplate<Scalar>::Matrix3X& rhs) const {
  CHECK_GT(rhs.cols(), 0);
  return q_A_B_.rotateVectorized(rhs).colwise() + A_t_A_B_;
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector3
QuatTransformationTemplate<Scalar>::operator*(
    const typename QuatTransformationTemplate<Scalar>::Vector3& rhs) const {
  return transform(rhs);
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector4
QuatTransformationTemplate<Scalar>::transform4(
    const typename QuatTransformationTemplate<Scalar>::Vector4& rhs) const {
  QuatTransformationTemplate<Scalar>::Vector4 rval;
  rval[3] = rhs[3];
  rval.template head<3>() =
      q_A_B_.rotate(rhs.template head<3>()) + rhs[3]*A_t_A_B_;
  return rval;
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector3
QuatTransformationTemplate<Scalar>::inverseTransform(
    const Vector3& rhs) const {
  return q_A_B_.inverseRotate(rhs - A_t_A_B_);
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector4
QuatTransformationTemplate<Scalar>::inverseTransform4(
    const typename QuatTransformationTemplate<Scalar>::Vector4& rhs) const {
  typename QuatTransformationTemplate<Scalar>::Vector4 rval;
  rval.template head<3>() = q_A_B_.inverseRotate(rhs.template head<3>() -
                                                 A_t_A_B_*rhs[3]);
  rval[3] = rhs[3];
  return rval;
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>
QuatTransformationTemplate<Scalar>::inverse() const {
  return QuatTransformationTemplate<Scalar>(q_A_B_.inverse(), -q_A_B_.inverseRotate(A_t_A_B_));
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>
QuatTransformationTemplate<Scalar>::inverted() const {
  return inverse();
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector6
QuatTransformationTemplate<Scalar>::log() const {
  return log(*this);
}

template<typename Scalar>
QuatTransformationTemplate<Scalar> QuatTransformationTemplate<Scalar>::exp(const Vector6& vec) {
  return QuatTransformationTemplate<Scalar>(vec);
}

template<typename Scalar>
typename QuatTransformationTemplate<Scalar>::Vector6
QuatTransformationTemplate<Scalar>::log(const QuatTransformationTemplate<Scalar>& T) {
  AngleAxisTemplate<Scalar> angleaxis(T.q_A_B_);
  return (Vector6() << T.A_t_A_B_, T.q_A_B_.log()).finished();
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>&
QuatTransformationTemplate<Scalar>::setRandom() {
  q_A_B_.setRandom();
  A_t_A_B_.setRandom();
  return *this;
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>&
QuatTransformationTemplate<Scalar>::setRandom(Scalar norm_translation) {
  setRandom();
  A_t_A_B_.normalize();
  A_t_A_B_ *= norm_translation;
  return *this;
}

template<typename Scalar>
QuatTransformationTemplate<Scalar>&
QuatTransformationTemplate<Scalar>::setRandom(Scalar norm_translation, Scalar angle_rad) {
  q_A_B_.setRandom(angle_rad);
  A_t_A_B_.setRandom().normalize();
  A_t_A_B_ *= norm_translation;
  return *this;
}

template<typename Scalar>
std::ostream & operator<<(std::ostream & out,
                          const QuatTransformationTemplate<Scalar>& pose) {
  out << pose.getTransformationMatrix();
  return out;
}

template<typename Scalar>
bool QuatTransformationTemplate<Scalar>::operator==(
    const QuatTransformationTemplate<Scalar>& rhs) const {
  return q_A_B_ == rhs.q_A_B_ && A_t_A_B_ == rhs.A_t_A_B_;
}

template <typename Scalar>
QuatTransformationTemplate<Scalar> QuatTransformationTemplate<
    Scalar>::constructAndRenormalizeRotation(const TransformationMatrix& T) {
  return QuatTransformationTemplate<Scalar>(
      Rotation::constructAndRenormalize(
          T.template topLeftCorner<3, 3>().eval()),
      T.template topRightCorner<3, 1>().eval());
}

template <typename Scalar>
template <typename ScalarAfterCast>
QuatTransformationTemplate<ScalarAfterCast>
QuatTransformationTemplate<Scalar>::cast() const {
  return QuatTransformationTemplate<ScalarAfterCast>(
      getRotation().template cast<ScalarAfterCast>(),
      getPosition().template cast<ScalarAfterCast>());
}

template<typename Scalar>
QuatTransformationTemplate<Scalar> interpolateComponentwise(
    const QuatTransformationTemplate<Scalar>& T_a,
    const QuatTransformationTemplate<Scalar>& T_b, const double lambda) {
  CHECK_GE(lambda, 0.0);
  CHECK_LE(lambda, 1.0);
  const PositionTemplate<Scalar> p_int =
      T_a.getPosition() + lambda * (T_b.getPosition() - T_a.getPosition());
  const Eigen::Quaternion<Scalar> q_int =
      T_a.getEigenQuaternion().slerp(lambda, T_b.getEigenQuaternion());
  return QuatTransformationTemplate<Scalar>(q_int, p_int);
}

} // namespace minimal
} // namespace kindr
#endif  // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
