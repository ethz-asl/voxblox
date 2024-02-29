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
#ifndef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#define KINDR_MINIMAL_QUAT_TRANSFORMATION_H_

#include <ostream>

#include <kindr/minimal/rotation-quaternion.h>
#include <kindr/minimal/position.h>

namespace kindr {
namespace minimal {

/// \class QuatTransformation
/// \brief A frame transformation built from a quaternion and a point
///
/// This transformation takes points from frame B to frame A, written
/// as \f${}_{A}\mathbf{p} = \mathbf{T}_{AB} {}_{B}\mathbf{p}\f$
///
/// In code, we write:
///
/// \code{.cpp}
/// A_p = T_A_B.transform(B_p);
/// \endcode
///
template <typename Scalar>
class QuatTransformationTemplate {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<Scalar, 6, 1> Vector6;

  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

  typedef PositionTemplate<Scalar> Position;
  typedef RotationQuaternionTemplate<Scalar> Rotation;
  typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;
  typedef Eigen::Matrix<Scalar, 4, 4> TransformationMatrix;

  /// \brief Constructor of identity transformation.
  QuatTransformationTemplate();
  QuatTransformationTemplate(const QuatTransformationTemplate& other) = default;

  explicit QuatTransformationTemplate(
      const RotationQuaternionTemplate<Scalar>& q_A_B, const Position& A_t_A_B);
  explicit QuatTransformationTemplate(
      const typename Rotation::Implementation& q_A_B, const Position& A_t_A_B);

  explicit QuatTransformationTemplate(
      const Position& A_t_A_B, const Rotation& q_A_B);
  explicit QuatTransformationTemplate(
      const Position& A_t_A_B, const typename Rotation::Implementation& q_A_B);

  explicit QuatTransformationTemplate(const TransformationMatrix& T);

  /// \brief a constructor based on the exponential map.
  /// translational part in the first 3 dimensions,
  /// rotational part in the last 3 dimensions.
  QuatTransformationTemplate(const Vector6& x_t_r);

  ~QuatTransformationTemplate();

  void setIdentity();

  /// \brief set to random transformation.
  QuatTransformationTemplate<Scalar>& setRandom();

  /// \brief set to random transformation with a given translation norm.
  QuatTransformationTemplate<Scalar>& setRandom(Scalar norm_translation);

  /// \brief set to random transformation with a given translation norm and rotation angle.
  QuatTransformationTemplate<Scalar>& setRandom(Scalar norm_translation, Scalar angle_rad);

  /// \brief get the position component.
  Position& getPosition();

  /// \brief get the position component.
  const Position& getPosition() const;

  /// \brief get the rotation component.
  Rotation& getRotation();

  /// \brief get the rotation component.
  const Rotation& getRotation() const;

  /// \brief get the rotation component as an Eigen Quaternion directly.
  const Eigen::Quaternion<Scalar>& getEigenQuaternion() const;

  /// \brief get the transformation matrix.
  TransformationMatrix getTransformationMatrix() const;

  /// \brief get the rotation matrix.
  RotationMatrix getRotationMatrix() const;

  /// \brief get the quaternion of rotation and the position as a vector.
  ///  [w x y z, x y z]
  Eigen::Matrix<Scalar, 7, 1> asVector() const;

  /// \brief compose two transformations.
  QuatTransformationTemplate<Scalar> operator*(
      const QuatTransformationTemplate<Scalar>& rhs) const;

  /// \brief transform a point.
  Vector3 operator*(const Vector3& rhs) const;

  /// \brief transform a point.
  Vector3 transform(const Vector3& rhs) const;

  /// \brief transform points.
  Matrix3X transformVectorized(const Matrix3X& rhs) const;

  /// \brief transform a point.
  Vector4 transform4(const Vector4& rhs) const;

  /// \brief transform a point by the inverse.
  Vector3 inverseTransform(const Vector3& rhs) const;

  /// \brief transform a point by the inverse.
  Vector4 inverseTransform4(const Vector4& rhs) const;

  /// \brief get the logarithmic map of the transformation
  /// note: this is the log map of SO(3)xR(3) and not SE(3)
  /// \return vector form of log map with first 3 components the translational
  ///         part and the last three the rotational part.
  Vector6 log() const;

  /// \brief get the exponential map of the parameters, resulting in a valid
  /// transformation note: this is the exp map of SO(3)xR(3) and not SE(3)
  /// \param[in] vec vector form of log map with first 3 components the translational
  ///                part and the last three the rotational part.
  /// \return The corresponding Transformation.
  static QuatTransformationTemplate<Scalar> exp(const Vector6& vec);

  /// \brief get the logarithmic map of the transformation
  /// note: this is the log map of SO(3)xR(3) and not SE(3)
  /// \return vector form of log map with first 3 components the translational
  ///         part and the last three the rotational part.
  static Vector6 log(const QuatTransformationTemplate<Scalar>& vec);

  /// \brief return a copy of the transformation inverted.
  QuatTransformationTemplate<Scalar> inverse() const;

  /// \deprecated use inverse() instead.
  QuatTransformationTemplate<Scalar> inverted() const __attribute__((deprecated));

  /// \brief check for binary equality.
  bool operator==(const QuatTransformationTemplate<Scalar>& rhs) const;

  /// \brief Factory to construct a QuatTransformTemplate from a transformation
  ///        matrix with a near orthonormal rotation matrix.
  static QuatTransformationTemplate<Scalar>
  constructAndRenormalizeRotation(const TransformationMatrix& T);

  /// \brief cast scalar elements to another type
  template <typename ScalarAfterCast>
  QuatTransformationTemplate<ScalarAfterCast> cast() const;

 private:
  /// The quaternion that takes vectors from B to A.
  ///
  /// \code{.cpp}
  /// A_v = q_A_B_.rotate(B_v);
  /// \endcode
  Rotation q_A_B_;
  /// The vector from the origin of A to the origin of B
  /// expressed in A
  Position A_t_A_B_;
};

typedef QuatTransformationTemplate<double> QuatTransformation;

template<typename Scalar>
std::ostream & operator<<(std::ostream & out,
                          const QuatTransformationTemplate<Scalar>& pose);

// Exponential interpolation (i.e., Slerp) in SO(3) and linear interpolation in
// R3. Lambda is in [0, 1], with 0 returning T_a, and 1 returning T_b.
template<typename Scalar>
inline QuatTransformationTemplate<Scalar> interpolateComponentwise(
    const QuatTransformationTemplate<Scalar>& T_a,
    const QuatTransformationTemplate<Scalar>& T_b, const double lambda);
} // namespace minimal
} // namespace kindr

#include <kindr/minimal/implementation/quat-transformation-inl.h>

#endif  // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
