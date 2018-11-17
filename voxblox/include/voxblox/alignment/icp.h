/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef VOXBLOX_ALIGNMENT_ICP_H_
#define VOXBLOX_ALIGNMENT_ICP_H_

#include <algorithm>
#include <memory>
#include <thread>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/interpolator/interpolator.h"
#include "voxblox/utils/approx_hash_array.h"

namespace voxblox {

/**
 * A class that performs point matching in an ICP like fashion to align a
 * pointcloud with the existing TSDF information. Note the process is slightly
 * different to traditional ICP:\n
 * 1) A "mini batch" of points is selected and the transform that gives the
 * minimum least squares error for their alignment is found.\n
 * 2) The "information" contained in this alignment is estimated and used to
 * fuse this refined transform with the initial guess.\n
 * 3) The process is repeated with a new mini batch of points until all points
 * in the pointcloud have been used.\n
 * This scheme does not really iterate and uses the TSDF distances instead of
 * building a kdtree. Both choices limit the capture region and so so assumes
 * the initial guess is reasonably accurate. However, these limitations allow
 * efficient correspondence estimation and parallelization, allowing efficient
 * real time performance.
 */
class ICP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Contains all the information needed to setup the ICP class.
  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool refine_roll_pitch = false;
    /**
     *  Number of points used in each alignment step. To allow simple threading
     *   the ICP process is split up into a large number of separate alignments
     *  performed on small pointclouds. This parameter dictates how many points
     *  are used in each "mini batch". The result are then combined weighting
     *  them by an estimate of the information gained by the alignment.
     */
    int mini_batch_size = 20;
    /**
     * Ratio of points that must lie within the truncation distance of an
     * allocated voxel
     */
    FloatingPoint min_match_ratio = 0.8;
    /// Ratio of points used in the ICP matching
    FloatingPoint subsample_keep_ratio = 0.5;
    /**
     * Weighting applied to the translational component of the initial guess.
     * Very roughly corresponds to the inverse covariance of the initial guess
     * multiplied by the variance in a measured points accuracy.
     */
    FloatingPoint inital_translation_weighting = 100.0;
    /**
     * Weighting applied to the rotational component of the initial guess.
     * See inital_translation_weighting for further details
     */
    FloatingPoint inital_rotation_weighting = 100.0;
    size_t num_threads = std::thread::hardware_concurrency();
  };

  /**
   * A normal member taking two arguments and returning an integer value.
   * @param config struct holding all relevant ICP parameters.
   */
  explicit ICP(const Config& config);

  /**
   * Runs the ICP method to align the points with the tsdf_layer.
   * @return the number of mini batches that were successful.
   */
  size_t runICP(const Layer<TsdfVoxel>& tsdf_layer, const Pointcloud& points,
                const Transformation& inital_T_tsdf_sensor,
                Transformation* refined_T_tsdf_sensor,
                const unsigned seed = std::chrono::system_clock::now()
                                          .time_since_epoch()
                                          .count());

  bool refiningRollPitch() { return config_.refine_roll_pitch; }

 private:
  typedef Transformation::Vector6 Vector6;

  /**
   * Calculates the rotation that will give the least mean-squared error
   * in aligning two sets of matched points. Assumes the pointclouds have the
   * same mean.
   * @param dim dimensionality of the alignment. dim = 2 performs yaw only
   * alignment, dim = 3 performs full 3 dof alignment, all other values are
   * invalid.
   * @return true if a valid rotation matrix was calculated, false otherwise.
   */
  template <size_t dim>
  static bool getRotationFromMatchedPoints(const PointsMatrix& src_demean,
                                           const PointsMatrix& tgt_demean,
                                           Rotation* R_tgt_src) {
    static_assert((dim == 3) || (dim == 2),
                  "Rotation calculation is only meaningful for 2D or 3D data");
    CHECK_NOTNULL(R_tgt_src);

    SquareMatrix<3> rotation_matrix = SquareMatrix<3>::Identity();

    SquareMatrix<dim> H =
        src_demean.topRows<dim>() * tgt_demean.topRows<dim>().transpose();

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<SquareMatrix<dim>> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    SquareMatrix<dim> u = svd.matrixU();
    SquareMatrix<dim> v = svd.matrixV();

    // Compute R = V * U'
    if (u.determinant() * v.determinant() < 0.0) {
      v.col(dim - 1) *= -1.0;
    }

    rotation_matrix.topLeftCorner<dim, dim>() = v * u.transpose();

    *R_tgt_src = Rotation(rotation_matrix);

    // not caught by is valid check
    if (!std::isfinite(rotation_matrix.sum())) {
      return false;
    }

    return Rotation::isValidRotationMatrix(rotation_matrix);
  }

  /**
   * Finds transform between a set of matching points that minimizes the
   * least mean-squared error.
   * @return true if T_tsdf_sensor was successfully estimated, false otherwise.
   */
  static bool getTransformFromMatchedPoints(const PointsMatrix& src,
                                            const PointsMatrix& tgt,
                                            const bool refine_roll_pitch,
                                            Transformation* T_tsdf_sensor);

  /**
   * A measure that vaguely indicates the amount of information a point
   *  contributes to the alignment estimate. The result can be taken as an
   * approximation of the Hessian, "Normalized" as the uncertainty in the points
   * is not used in its computation. Note this cannot be used as an absolute
   * estimate of alignment quality as even highly self-similar environments will
   * report sub-millimeter errors due to the baked in assumption that all
   * correspondences are correct.
   * @param info_vector vector representing the information over the 6-dof pose.
   * The points information is added to the information existing in this vector.
   */
  static void addNormalizedPointInfo(const Point& point,
                                     const Point& normalized_point_normal,
                                     Vector6* info_vector);

  /// Generates a set of matching points from a pointcloud and tsdf layer.
  void matchPoints(const Pointcloud& points, const size_t start_idx,
                   const Transformation& T_tsdf_sensor, PointsMatrix* src,
                   PointsMatrix* tgt, Vector6* info_vector);

  /**
   * Performs one mini batch step and gives the refined transform.
   * @return true if a valid transform was generated
   */
  bool stepICP(const Pointcloud& points, const size_t start_idx,
               const Transformation& inital_T_tsdf_sensor,
               Transformation* refined_T_tsdf_sensor, Vector6* info_vector);

  /**
   * A thread safe function that will continually process points until the
   * alignment is finished. Called by runICP.
   */
  void runThread(const Pointcloud& points,
                 Transformation* current_T_tsdf_sensor,
                 Vector6* base_info_vector, size_t* num_updates);

  Config config_;

  std::atomic<size_t> atomic_idx_;
  std::mutex mutex_;

  FloatingPoint voxel_size_;
  FloatingPoint voxel_size_inv_;
  std::shared_ptr<Interpolator<TsdfVoxel>> interpolator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ALIGNMENT_ICP_H_
