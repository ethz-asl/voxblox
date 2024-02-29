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
#ifndef KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
#define KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
#include <kindr/minimal/angle-axis.h>
#include <kindr/minimal/rotation-quaternion.h>
#include <glog/logging.h>

namespace kindr {
namespace minimal {

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate() :
    C_A_B_(Implementation::Identity()) {
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(
    Scalar w, Scalar x, Scalar y, Scalar z) : C_A_B_(w, Vector3(x,y,z)) {
  CHECK_NEAR(Vector3(x,y,z).squaredNorm(), static_cast<Scalar>(1.0),
             static_cast<Scalar>(1e-4));
}
 
template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(
    Scalar angle, const typename AngleAxisTemplate<Scalar>::Vector3& axis) :
    C_A_B_(angle, axis){
    CHECK_NEAR(axis.squaredNorm(), static_cast<Scalar>(1.0),
               static_cast<Scalar>(1e-4));
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(const Implementation& angleAxis) :
    C_A_B_(angleAxis) {
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(const RotationMatrix& matrix) :
    C_A_B_(matrix) {
  // \todo furgalep Check that the matrix was good...
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(const Vector3& rotationVector) {
    const Scalar angle_rad = rotationVector.norm();
    if (angle_rad < std::numeric_limits<Scalar>::epsilon()) {
      setIdentity();
    } else {
      Vector3 axis = rotationVector;
      axis = axis / angle_rad;

      C_A_B_ = Implementation(angle_rad, axis);
    }
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::AngleAxisTemplate(
    const RotationQuaternionTemplate<Scalar>& quat) :
    C_A_B_(quat.toImplementation()) {
}

template<typename Scalar>
AngleAxisTemplate<Scalar>::~AngleAxisTemplate() { }

template<typename Scalar>
AngleAxisTemplate<Scalar>& AngleAxisTemplate<Scalar>::operator=(
    const AngleAxisTemplate<Scalar>& rhs) {
  if(this != &rhs) {
    C_A_B_ = rhs.C_A_B_;
  }
  return *this;
}

template<typename Scalar>
Scalar AngleAxisTemplate<Scalar>::angle() const{
  return C_A_B_.angle();
}

template<typename Scalar>
void AngleAxisTemplate<Scalar>::setAngle(Scalar angle){
  C_A_B_.angle() = angle;
}

template<typename Scalar>
const typename AngleAxisTemplate<Scalar>::Vector3&
AngleAxisTemplate<Scalar>::axis() const{
  return C_A_B_.axis();
}

template<typename Scalar>
void AngleAxisTemplate<Scalar>::setAxis(const Vector3& axis){
  CHECK_NEAR(axis.squaredNorm(), static_cast<Scalar>(1.0),
             static_cast<Scalar>(1e-4));
  C_A_B_.axis() = axis;
}

template<typename Scalar>
void AngleAxisTemplate<Scalar>::setAxis(Scalar v1, Scalar v2, Scalar v3){
  C_A_B_.axis() = Vector3(v1,v2,v3);
  CHECK_NEAR(C_A_B_.axis().squaredNorm(), static_cast<Scalar>(1.0),
             static_cast<Scalar>(1e-4));
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Vector4
AngleAxisTemplate<Scalar>::vector() const{
    Vector4 vector;
    vector(0) = angle();
    vector.template tail<3>() = C_A_B_.axis();
    return vector;
}

template<typename Scalar>
AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::getUnique() const {
  // first wraps angle into [-pi,pi)
  AngleAxisTemplate aa(fmod(angle()+M_PI, 2*M_PI)-M_PI, C_A_B_.axis());
    if(aa.angle() > 0)  {
      return aa;
    } else if(aa.angle() < 0) {
      if(aa.angle() != -M_PI) {
        return AngleAxisTemplate(-aa.angle(), -aa.axis());
      } else { // angle == -pi, so axis must be viewed further, because -pi,axis
               // does the same as -pi,-axis.
        if(aa.axis()[0] < 0) {
          return AngleAxisTemplate(-aa.angle(), -aa.axis());
        } else if(aa.axis()[0] > 0) {
          return AngleAxisTemplate(-aa.angle(), aa.axis());
        } else { // v1 == 0
          if(aa.axis()[1] < 0) {
            return AngleAxisTemplate(-aa.angle(), -aa.axis());
          } else if(aa.axis()[1] > 0) {
            return AngleAxisTemplate(-aa.angle(), aa.axis());
          } else { // v2 == 0
            if(aa.axis()[2] < 0) { // v3 must be -1 or 1
              return AngleAxisTemplate(-aa.angle(), -aa.axis());
            } else  {
              return AngleAxisTemplate(-aa.angle(), aa.axis());
            }
          }
        }
      }
    } else { // angle == 0
      return AngleAxisTemplate();
    }
}

template<typename Scalar>
AngleAxisTemplate<Scalar>& AngleAxisTemplate<Scalar>::setUnique() {
  *this = getUnique();
  return *this;
}

template<typename Scalar>
AngleAxisTemplate<Scalar>& AngleAxisTemplate<Scalar>::setIdentity() {
  C_A_B_ = C_A_B_.Identity();
  return *this;
}

template<typename Scalar>
AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::inverse() const {
  return AngleAxisTemplate(C_A_B_.inverse());
}

template<typename Scalar>
AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::inverted() const {
  return inverse();
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Vector3 AngleAxisTemplate<Scalar>::rotate(
    const AngleAxisTemplate<Scalar>::Vector3& v) const {
  return C_A_B_*v;
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Vector4
AngleAxisTemplate<Scalar>::rotate4(
    const AngleAxisTemplate<Scalar>::Vector4& v) const {
  AngleAxisTemplate<Scalar>::Vector4 vprime;
  vprime[3] = v[3];
  vprime.template head<3>() = C_A_B_ * v.template head<3>();
  return vprime;
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Vector3
AngleAxisTemplate<Scalar>::inverseRotate(
    const AngleAxisTemplate<Scalar>::Vector3& v) const {
  return C_A_B_.inverse() * v;
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Vector4
AngleAxisTemplate<Scalar>::inverseRotate4(
    const typename AngleAxisTemplate<Scalar>::Vector4& v) const {
  Eigen::Vector4d vprime;
  vprime[3] = v[3];
  vprime.template head<3>() = C_A_B_.inverse() * v.template head<3>();
  return vprime;
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::Implementation&
AngleAxisTemplate<Scalar>::toImplementation() {
  return C_A_B_;
}

template<typename Scalar>
const typename AngleAxisTemplate<Scalar>::Implementation&
AngleAxisTemplate<Scalar>::toImplementation() const {
  return C_A_B_;
}

template<typename Scalar>
AngleAxisTemplate<Scalar>& AngleAxisTemplate<Scalar>::normalize() {
  C_A_B_.axis().normalize();
  return *this;
}

template<typename Scalar>
AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::operator*(
    const AngleAxisTemplate& rhs) const {
  return AngleAxisTemplate(Implementation(C_A_B_ * rhs.C_A_B_));
}

template<typename Scalar>
Scalar AngleAxisTemplate<Scalar>::getDisparityAngle(
    const AngleAxisTemplate& rhs) const {
  return (rhs * this->inverse()).getUnique().angle();
}

template<typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const AngleAxisTemplate<Scalar>& rhs) {
  out << rhs.vector().transpose();
  return out;
}

template<typename Scalar>
typename AngleAxisTemplate<Scalar>::RotationMatrix
AngleAxisTemplate<Scalar>::getRotationMatrix() const {
  return C_A_B_.matrix();
}

} // namespace minimal
} // namespace kindr
#endif  // KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
