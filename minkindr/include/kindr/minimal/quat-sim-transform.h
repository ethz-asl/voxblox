#ifndef KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
#define KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_

#include <ostream>

#include <Eigen/Dense>

#include "kindr/minimal/quat-transformation.h"

namespace kindr {
namespace minimal {

// Scale & rotate then translate = scale then transform.
// In particular, the transformation matrix is:
//
//   R*s t     R t   s*I 0
//   0   1  =  0 1 * 0   1
template <typename Scalar>
class QuatSimTransformTemplate {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef QuatTransformationTemplate<Scalar> Transform;
  typedef QuatSimTransformTemplate<Scalar> Sim3;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 7, 1> Vector7;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

  // Creates identity similarity transform.
  QuatSimTransformTemplate();

  QuatSimTransformTemplate(const Transform& T_A_B, const Scalar scale_A_B);

  QuatSimTransformTemplate(const Vector7& log_vector);

  inline Vector3 operator*(const Vector3& rhs) const;

  // Vectorized, applies operator * to each column vector.
  inline Matrix3X operator*(const Matrix3X& rhs) const;

  inline Sim3 operator*(const Sim3& rhs) const;

  inline Sim3 operator*(const Transform& rhs) const;

  inline Sim3 inverse() const;

  inline Vector7 log() const;

  inline Eigen::Matrix<Scalar, 4, 4> getTransformationMatrix() const;
  inline const Transform& getTransform() const { return T_A_B_; }
  inline Scalar getScale() const { return scale_A_B_; }

  inline void setScale(const Scalar scale_A_B) { scale_A_B_ = scale_A_B; }

private:
  Transform T_A_B_;
  Scalar scale_A_B_;

  template <typename FriendScalar>
  friend std::ostream & operator<<(
      std::ostream &, const QuatSimTransformTemplate<FriendScalar>&);
};

typedef QuatSimTransformTemplate<double> QuatSimTransform;

template<typename Scalar>
inline QuatSimTransformTemplate<Scalar> operator*(
    const QuatTransformationTemplate<Scalar>& lhs,
    const QuatSimTransformTemplate<Scalar>& rhs);

template<typename Scalar>
std::ostream & operator<<(std::ostream & out,
                          const QuatSimTransformTemplate<Scalar>& sim_3);

}  // namespace minimal
}  // namespace kindr

#include "kindr/minimal/implementation/quat-sim-transform-inl.h"

#endif  // KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
