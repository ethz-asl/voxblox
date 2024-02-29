#pragma once

#ifndef MINKINDR_CONVERSIONS_KINDR_TF_H
#define MINKINDR_CONVERSIONS_KINDR_TF_H

#include <kindr/minimal/quat-transformation.h>
#include <tf2/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.hpp> // #include <tf_conversions/tf_eigen.h>

namespace tf_conversions{

  inline void quaternionTFToEigen(const tf2::Quaternion& t, Eigen::Quaterniond& e)
  {
    e = Eigen::Quaterniond(t[3],t[0],t[1],t[2]);
  }

  inline void quaternionEigenToTF(const Eigen::Quaterniond& e, tf2::Quaternion& t)
  {
    t[0] = e.x();
    t[1] = e.y();
    t[2] = e.z();
    t[3] = e.w();
  }

  inline void vectorTFToEigen(const tf2::Vector3& t, Eigen::Vector3d& e)
  {
    e(0) = t[0];
    e(1) = t[1];
    e(2) = t[2];
  }

  inline void vectorEigenToTF(const Eigen::Vector3d& e, tf2::Vector3& t)
  {
    t[0] = e(0);
    t[1] = e(1);
    t[2] = e(2);
  }

}

namespace tf2 {

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void quaternionKindrToTF(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& kindr,
    tf2::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf_conversions::quaternionEigenToTF(kindr.toImplementation(), *tf_type);
}

template <typename Scalar>
void quaternionTFToKindr(
    const tf2::Quaternion& tf_type,
    kindr::minimal::RotationQuaternionTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  tf_conversions::quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}
// Also the Eigen implementation version of this.
template <typename Scalar>
void quaternionKindrToTF(const Eigen::Quaternion<Scalar>& kindr,
                         tf2::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf_conversions::quaternionEigenToTF(kindr, *tf_type);
}

template <typename Scalar>
void quaternionTFToKindr(const tf2::Quaternion& tf_type,
                         Eigen::Quaternion<Scalar>* kindr) {

  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond kindr_double;
  tf_conversions::quaternionTFToEigen(tf_type, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void vectorKindrToTF(const Eigen::Matrix<Scalar, 3, 1>& kindr,
                     tf2::Vector3* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf_conversions::vectorEigenToTF(kindr, *tf_type);
}

template <typename Scalar>
void vectorTFToKindr(const tf2::Vector3& tf_type,
                     Eigen::Matrix<Scalar, 3, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<double, 3, 1> kindr_double;
  tf_conversions::vectorTFToEigen(tf_type, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::msg::Transform.
template <typename Scalar>
void transformKindrToTF(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    tf2::Transform* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Vector3 origin;
  tf2::Quaternion rotation;
  vectorKindrToTF(kindr.getPosition(), &origin);
  quaternionKindrToTF(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

template <typename Scalar>
void transformTFToKindr(
    const tf2::Transform& tf_type,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionTFToKindr(tf_type.getRotation(), &rotation);
  vectorTFToKindr(tf_type.getOrigin(), &position);

  // Enforce positive w.
  if (rotation.w() < 0) {
    rotation.coeffs() = -rotation.coeffs();
  }

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

}  // namespace tf2

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
