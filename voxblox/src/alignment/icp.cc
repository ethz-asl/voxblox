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

#include <random>

#include "voxblox/alignment/icp.h"

namespace voxblox {

ICP::ICP(const Config& config) : config_(config) {}

bool ICP::getTransformFromMatchedPoints(const PointsMatrix& src,
                                        const PointsMatrix& tgt,
                                        const bool refine_roll_pitch,
                                        Transformation* T) {
  CHECK(src.cols() == tgt.cols());

  // find and remove mean
  const Point src_center = src.rowwise().mean();
  const Point tgt_center = tgt.rowwise().mean();

  const PointsMatrix src_demean = src.colwise() - src_center;
  const PointsMatrix tgt_demean = tgt.colwise() - tgt_center;

  Matrix3 rotation_matrix = Matrix3::Identity();

  if (!refine_roll_pitch) {
    // Assemble the correlation matrix H = source * target'
    Matrix2 H = src_demean.topRows<2>() * tgt_demean.topRows<2>().transpose();

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Matrix2> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix2 u = svd.matrixU();
    Matrix2 v = svd.matrixV();

    // Compute R = V * U'
    if (u.determinant() * v.determinant() < 0.0) {
      v.col(1) *= -1.0;
    }

    rotation_matrix.topLeftCorner<2, 2>() = v * u.transpose();
  } else {
    // Assemble the correlation matrix H = source * target'
    Matrix3 H = src_demean * tgt_demean.transpose();

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Matrix3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3 u = svd.matrixU();
    Matrix3 v = svd.matrixV();

    // Compute R = V * U'
    if (u.determinant() * v.determinant() < 0) {
      v.col(2) *= -1.0;
    }

    rotation_matrix = v * u.transpose();
  }

  Point trans = tgt_center - (rotation_matrix * src_center);

  if (!Rotation::isValidRotationMatrix(rotation_matrix)) {
    return false;
  }

  *T = Transformation(Rotation(rotation_matrix), trans);
  return true;
}

void ICP::matchPoints(const Pointcloud& points, const Transformation& T,
                      PointsMatrix* src, PointsMatrix* tgt) {
  ThreadSafeIndex index_getter(points.size());

  std::list<std::thread> threads;
  src->resize(3, points.size());
  tgt->resize(3, points.size());

  std::atomic<size_t> atomic_out_idx(0);

  // get distance info
  for (size_t i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(&ICP::calcMatches, this, points, T, &index_getter,
                         &atomic_out_idx, src, tgt);
  }

  for (std::thread& thread : threads) {
    thread.join();
  }

  src->conservativeResize(3, atomic_out_idx.load());
  tgt->conservativeResize(3, atomic_out_idx.load());
}

void ICP::subSample(const Pointcloud& points_in, Pointcloud* points_out) {
  const int num_keep = config_.subsample_keep_ratio * points_in.size();
  const int skip_per_keep = points_in.size() / num_keep;

  std::default_random_engine generator(0);
  std::uniform_int_distribution<int> distribution(
      0, std::max(0, skip_per_keep - 1));

  points_out->reserve(num_keep);

  for (int i = 0; i < num_keep; ++i) {
    const int point_idx = i * skip_per_keep + distribution(generator);
    points_out->push_back(points_in[point_idx]);
  }
}

void ICP::calcMatches(const Pointcloud& points, const Transformation& T,
                      ThreadSafeIndex* index_getter,
                      std::atomic<size_t>* atomic_out_idx, PointsMatrix* src,
                      PointsMatrix* tgt) {
  constexpr bool kInterpolateDist = false;
  constexpr bool kInterpolateGrad = false;
  constexpr FloatingPoint kMinGradMag = 0.1;

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point = points[point_idx];
    const Point point_tformed = T * point;
    const AnyIndex voxel_idx =
        getGridIndexFromPoint<AnyIndex>(point_tformed, voxel_size_inv_);

    FloatingPoint distance;
    Point gradient;

    if (interpolator_->getDistance(point_tformed, &distance,
                                   kInterpolateDist) &&
        interpolator_->getGradient(point_tformed, &gradient,
                                   kInterpolateGrad) &&
        (gradient.squaredNorm() > kMinGradMag)) {
      gradient.normalize();

      // uninterpolated distance is to the center of the voxel, as we now have
      // the gradient we can use it to do better
      const Point voxel_center =
          getCenterPointFromGridIndex<AnyIndex>(voxel_idx, voxel_size_);
      distance += gradient.dot(point_tformed - voxel_center);

      const size_t idx = atomic_out_idx->fetch_add(1);
      src->col(idx) = point_tformed;
      tgt->col(idx) = point_tformed - distance * gradient;
    }
  }
}

bool ICP::stepICP(const Pointcloud& points, const Transformation& T_in,
                  Transformation* T_out) {
  PointsMatrix src;
  PointsMatrix tgt;

  matchPoints(points, T_in, &src, &tgt);

  if (src.cols() <
      std::max(3, static_cast<int>(points.size() * config_.min_match_ratio))) {
    return false;
  }

  Transformation T_delta;
  if (!getTransformFromMatchedPoints(src, tgt, config_.refine_roll_pitch,
                                     &T_delta)) {
    return false;
  }
  *T_out = T_delta * T_in;

  return true;
}

bool ICP::runICP(const Layer<TsdfVoxel>& tsdf_layer, const Pointcloud& points,
                 const Transformation& T_in, Transformation* T_out) {
  if (config_.iterations == 0) {
    *T_out = T_in;
    return true;
  }

  interpolator_ = std::make_shared<Interpolator<TsdfVoxel>>(&tsdf_layer);
  voxel_size_ = tsdf_layer.voxel_size();
  voxel_size_inv_ = tsdf_layer.voxel_size_inv();

  Pointcloud subsampled_points;
  subSample(points, &subsampled_points);

  Transformation T_current = T_in;

  bool success = true;
  for (int i = 0; i < config_.iterations; ++i) {
    if (!stepICP(subsampled_points, T_current, T_out)) {
      *T_out = T_in;
      success = false;
      break;
    }

    T_current = *T_out;
  }

  interpolator_.reset();

  return success;
}

}  // namespace voxblox
