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
#ifndef KINDR_MIN_ROTATION_QUATERNION_H_
#define KINDR_MIN_ROTATION_QUATERNION_H_

#include <ostream>

#include <Eigen/Dense>

namespace kindr {
namespace minimal {

template<typename Scalar>
class AngleAxisTemplate;

/// \class RotationQuaternion
/// \brief a minimal implementation of a passive Hamiltonian rotation
///        (unit-length) quaternion
///
/// This rotation takes vectors from frame B to frame A, written
/// as \f${}_{A}\mathbf{v} = \mathbf{C}_{AB} {}_{B}\mathbf{v}\f$
///
/// In code, we write:
///
/// \code{.cpp}
/// A_v = q_A_B.rotate(B_v);
/// \endcode
///
template<typename Scalar>
class RotationQuaternionTemplate {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  typedef Vector3 Imaginary;

  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

  typedef Eigen::Quaternion<Scalar> Implementation;

  typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;

  /// \brief initialize to identity.
  RotationQuaternionTemplate();
  RotationQuaternionTemplate(const RotationQuaternionTemplate& other) = default;

  /// \brief initialize from angle scaled axis.
  explicit RotationQuaternionTemplate(const Vector3& angle_scaled_axis);

  /// \brief initialize from real and imaginary components (real first).
  explicit RotationQuaternionTemplate(Scalar w, Scalar x, Scalar y, Scalar z);

  /// \brief initialize from real and imaginary components.
  explicit RotationQuaternionTemplate(Scalar real, const Imaginary& imaginary);

  /// \brief initialize from an Eigen quaternion.
  explicit RotationQuaternionTemplate(const Implementation& quaternion);

  /// \brief initialize from a rotation matrix.
  explicit RotationQuaternionTemplate(const RotationMatrix& matrix);

  /// \brief take an approximate rotation matrix, recover the closest matrix
  /// in SO(3) and construct.
  static RotationQuaternionTemplate<Scalar> fromApproximateRotationMatrix(
      const RotationMatrix& matrix);

  /// \brief initialize from an AngleAxis.
  explicit RotationQuaternionTemplate(
      const AngleAxisTemplate<Scalar>& angleAxis);

  ~RotationQuaternionTemplate();

  /// \brief the real component of the quaternion.
  Scalar w() const;
  /// \brief the first imaginary component of the quaternion.
  Scalar x() const;
  /// \brief the second imaginary component of the quaternion.
  Scalar y() const;
  /// \brief the third imaginary component of the quaternion.
  Scalar z() const;

  /// \brief the imaginary components of the quaterion.
  Imaginary imaginary() const;

  /// \brief get the components of the quaternion as a vector (real first).
  Vector4 vector() const;

  /// \brief set the quaternion by its values (real, imaginary).
  void setValues(Scalar w, Scalar x, Scalar y, Scalar z);

  /// \brief set the quaternion by its real and imaginary parts.
  void setParts(Scalar real, const Imaginary& imag);

  /// \brief get a copy of the representation that is unique.
  RotationQuaternionTemplate<Scalar> getUnique() const;

  /// \brief set the quaternion to its unique representation.
  RotationQuaternionTemplate<Scalar>& setUnique();

  /// \brief set the quaternion to identity.
  RotationQuaternionTemplate<Scalar>& setIdentity();

  /// \brief set to random rotation.
  RotationQuaternionTemplate<Scalar>& setRandom();

  /// \brief set to random rotation with a given angle.
  RotationQuaternionTemplate<Scalar>& setRandom(Scalar angle_rad);

  /// \brief get a copy of the quaternion inverted.
  RotationQuaternionTemplate<Scalar> inverse() const;

  /// \deprecated use inverse instead.
  RotationQuaternionTemplate<Scalar> inverted() const __attribute__((deprecated));

  /// \brief get a copy of the conjugate of the quaternion.
  RotationQuaternionTemplate<Scalar> conjugated() const;

  /// \brief rotate a vector, v.
  Vector3 rotate(const Vector3& v) const;

  /// \brief rotate vectors v.
  Matrix3X rotateVectorized(const Matrix3X& v) const;

  /// \brief rotate a vector, v.
  Vector4 rotate4(const Vector4& v) const;

  /// \brief rotate a vector, v.
  Vector3 inverseRotate(const Vector3& v) const;

  /// \brief rotate a vector, v.
  Vector4 inverseRotate4(const Vector4& v) const;

  /// \brief cast to the implementation type.
  Implementation& toImplementation();

  /// \brief cast to the implementation type.
  const Implementation& toImplementation() const;

  /// \brief get the norm of the quaternion.
  Scalar norm() const;

  /// \brief get the squared norm of the quaternion.
  Scalar squaredNorm() const;

  /// \brief get the angle [rad] between this and the other quaternion.
  Scalar getDisparityAngle(const RotationQuaternionTemplate<Scalar>& rhs) const;

  /// \brief get the angle [rad] between this and the angle axis.
  Scalar getDisparityAngle(const AngleAxisTemplate<Scalar>& rhs) const;

  /// \brief enforce the unit length constraint.
  RotationQuaternionTemplate<Scalar>& normalize();

  /// \brief compose two quaternions.
  RotationQuaternionTemplate<Scalar> operator*(
      const RotationQuaternionTemplate<Scalar>& rhs) const;

  /// \brief compose quaternion and angle axis.
  RotationQuaternionTemplate<Scalar> operator*(
      const AngleAxisTemplate<Scalar>& rhs) const;

  /// \brief assignment operator.
  RotationQuaternionTemplate<Scalar>& operator=(
      const RotationQuaternionTemplate<Scalar>& rhs);

  /// \brief get the rotation matrix.
  RotationMatrix getRotationMatrix() const;

  /// \brief check for binary equality.
  bool operator==(const RotationQuaternionTemplate<Scalar>& rhs) const {
    return vector() == rhs.vector();
  }

  // Compute the matrix log of the quaternion.
  static Vector3 log(const RotationQuaternionTemplate<Scalar>& q);

  // Compute the matrix exponential of the quaternion.
  static RotationQuaternionTemplate<Scalar> exp(const Vector3& dx);

  Vector3 log() const;

  /// \brief Check the validity of a rotation matrix.
  static bool isValidRotationMatrix(const RotationMatrix& matrix);
  static bool isValidRotationMatrix(const RotationMatrix& matrix,
                                    const Scalar threshold);

  /// \brief Factory to construct a RotationQuaternionTemplate from a near
  ///        orthonormal rotation matrix.
  inline static RotationQuaternionTemplate<Scalar> constructAndRenormalize(
      const RotationMatrix& R) {
    return RotationQuaternionTemplate<Scalar>(Implementation(R).normalized());
  }

  /// \brief cast scalar elements to another type
  template <typename ScalarAfterCast>
  RotationQuaternionTemplate<ScalarAfterCast> cast() const;

 private:
  void normalizationHelper(Implementation* quaternion) const;

  Implementation q_A_B_;
};

typedef RotationQuaternionTemplate<double> RotationQuaternion;

template<typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const RotationQuaternionTemplate<Scalar>& rhs);

}  // namespace minimal
}  // namespace kindr

#include <kindr/minimal/implementation/rotation-quaternion-inl.h>

#endif  // KINDR_MIN_ROTATION_QUATERNION_H_
