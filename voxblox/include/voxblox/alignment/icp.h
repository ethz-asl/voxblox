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

class ICP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool refine_roll_pitch = false;
    int mini_batch_size = 20;
    FloatingPoint min_match_ratio = 0.8;
    FloatingPoint subsample_keep_ratio = 0.5;
    FloatingPoint inital_translation_weighting = 100.0;
    FloatingPoint inital_rotation_weighting = 100.0;
    size_t num_threads = std::thread::hardware_concurrency();
  };

  explicit ICP(const Config& config);

  // returns number of successful mini batches
  size_t runICP(const Layer<TsdfVoxel>& tsdf_layer, const Pointcloud& points,
                const Transformation& inital_T_tsdf_sensor,
                Transformation* refined_T_tsdf_sensor,
                const unsigned seed = std::chrono::system_clock::now()
                                          .time_since_epoch()
                                          .count());

  bool refiningRollPitch() { return config_.refine_roll_pitch; }

 private:
  typedef Transformation::Vector6 Vector6;

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

  static bool getTransformFromMatchedPoints(const PointsMatrix& src,
                                            const PointsMatrix& tgt,
                                            const bool refine_roll_pitch,
                                            Transformation* T_tsdf_sensor);

  static void addNormalizedPointInfo(const Point& point,
                                     const Point& normalized_point_normal,
                                     Vector6* info_vector);

  void matchPoints(const Pointcloud& points, const size_t start_idx,
                   const Transformation& T_tsdf_sensor, PointsMatrix* src,
                   PointsMatrix* tgt, Vector6* info_vector);

  bool stepICP(const Pointcloud& points, const size_t start_idx,
               const Transformation& inital_T_tsdf_sensor,
               Transformation* refined_T_tsdf_sensor, Vector6* info_vector);

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
