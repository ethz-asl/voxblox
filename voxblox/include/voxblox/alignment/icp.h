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
    int iterations = 5;
    FloatingPoint min_match_ratio = 0.5;
    FloatingPoint subsample_keep_ratio = 0.05;
    size_t num_threads = std::thread::hardware_concurrency();
  };

  explicit ICP(const Config& config);

  bool runICP(const Layer<TsdfVoxel>& tsdf_layer, const Pointcloud& points,
              const Transformation& T_in, Transformation* T_out);

  bool refiningRollPitch() { return config_.refine_roll_pitch; }

 private:
  static bool getTransformFromMatchedPoints(const PointsMatrix& src,
                                            const PointsMatrix& tgt,
                                            const bool refine_roll_pitch,
                                            Transformation* T);

  void subSample(const Pointcloud& points_in, Pointcloud* points_out);

  void matchPoints(const Pointcloud& points, const Transformation& T,
                   PointsMatrix* src, PointsMatrix* tgt);

  void calcMatches(const Pointcloud& points, const Transformation& T,
                   ThreadSafeIndex* index_getter,
                   std::atomic<size_t>* atomic_out_idx, PointsMatrix* src,
                   PointsMatrix* tgt);

  bool stepICP(const Pointcloud& points, const Transformation& T_in,
               Transformation* T_out);

  Config config_;

  FloatingPoint voxel_size_;
  FloatingPoint voxel_size_inv_;
  std::shared_ptr<Interpolator<TsdfVoxel>> interpolator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ALIGNMENT_ICP_H_
