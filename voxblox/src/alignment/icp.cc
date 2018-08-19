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
#include "voxblox/interpolator/interpolator.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

ICP::ICP(Config config) : config_(config) {}

bool ICP::getTransformFromCorrelation(const PointsMatrix &src_demean,
                                      const Point &src_center,
                                      const PointsMatrix &tgt_demean,
                                      const Point &tgt_center,
                                      Transformation *T) {
  CHECK(src_demean.cols() == tgt_demean.cols());

  // Assemble the correlation matrix H = source * target'
  Matrix3 H = src_demean * tgt_demean.transpose();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Matrix3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3 u = svd.matrixU();
  Matrix3 v = svd.matrixV();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0) {
    for (int x = 0; x < 3; ++x) v(x, 2) *= -1;
  }

  // Form transformation
  Matrix3 rotation_matrix = v * u.transpose();
  Point trans = tgt_center - (rotation_matrix * src_center);

  if (!Rotation::isValidRotationMatrix(rotation_matrix)) {
    return false;
  }

  *T = Transformation(Rotation(rotation_matrix), trans);
  return true;
}

bool ICP::getTransformFromMatchedPoints(const PointsMatrix &src,
                                        const PointsMatrix &tgt,
                                        Transformation *T) {
  CHECK(src.cols() == tgt.cols());

  // find and remove mean
  const Point src_center = src.rowwise().mean();
  const Point tgt_center = tgt.rowwise().mean();

  PointsMatrix src_demean = src.colwise() - src_center;
  PointsMatrix tgt_demean = tgt.colwise() - tgt_center;

  // align
  return getTransformFromCorrelation(src_demean, src_center, tgt_demean,
                                     tgt_center, T);
}

void ICP::matchPoints(const Layer<TsdfVoxel> *tsdf_layer,
                      const Pointcloud &points, const Transformation &T,
                      PointsMatrix *src, PointsMatrix *tgt) {
  ThreadSafeIndex index_getter(points.size());

  std::list<std::thread> threads;
  src->resize(3, points.size());
  tgt->resize(3, points.size());

  std::atomic<size_t> atomic_out_idx(0);

  for (size_t i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(&ICP::calcMatches, this, tsdf_layer, points, T, i,
                         &index_getter, &atomic_out_idx, src, tgt);
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  src->conservativeResize(3, atomic_out_idx.load());
  tgt->conservativeResize(3, atomic_out_idx.load());
}

void ICP::subSample(const Pointcloud &points_in, Pointcloud *points_out) {
  std::default_random_engine generator(0);
  std::uniform_real_distribution<float> distribution(0, 1);

  points_out->reserve(1.1 * config_.subsample_keep_ratio * points_in.size());
  for (const Point &point : points_in) {
    if (distribution(generator) < config_.subsample_keep_ratio) {
      points_out->push_back(point);
    }
  }
}

void ICP::calcMatches(const Layer<TsdfVoxel> *tsdf_layer,
                      const Pointcloud &points, const Transformation &T,
                      const size_t cache_idx, ThreadSafeIndex *index_getter,
                      std::atomic<size_t> *atomic_out_idx, PointsMatrix *src,
                      PointsMatrix *tgt) {
  Interpolator<TsdfVoxel> interpolator(tsdf_layer);

  constexpr bool kInterpolateDist = false;
  constexpr bool kInterpolateGrad = false;
  constexpr FloatingPoint kMinGradMag = 0.1;

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point &point = points[point_idx];
    const Point point_tformed = T * point;
    const AnyIndex voxel_idx = getGridIndexFromPoint<AnyIndex>(
        point_tformed, tsdf_layer->voxel_size_inv());

    std::shared_ptr<DistInfo> dist_info;

    DistInfoMap::const_iterator it =
        dist_info_caches_[cache_idx].find(voxel_idx);

    if (it != dist_info_caches_[cache_idx].end()) {
      dist_info = it->second;
    } else {
      dist_info = std::make_shared<DistInfo>();

      if (interpolator.getDistance(point_tformed, &dist_info->distance,
                                   kInterpolateDist) &&
          interpolator.getGradient(point_tformed, &dist_info->gradient,
                                   kInterpolateGrad) &&
          (dist_info->gradient.squaredNorm() > kMinGradMag)) {
        dist_info->gradient.normalize();
        dist_info->valid = true;
      } else {
        dist_info->valid = false;
      }

      dist_info_caches_[cache_idx][voxel_idx] = dist_info;
    }

    if (dist_info->valid) {
      // uninterpolated distance is to the center of the voxel, as we now have
      // the gradient we can use it to do better
      const Point voxel_center = getCenterPointFromGridIndex<AnyIndex>(
          voxel_idx, tsdf_layer->voxel_size());
      FloatingPoint distance =
          dist_info->distance +
          dist_info->gradient.dot(point_tformed - voxel_center);

      const size_t idx = atomic_out_idx->fetch_add(1);
      src->col(idx) = point;
      tgt->col(idx) = point_tformed - distance * dist_info->gradient;
    }
  }
}

bool ICP::stepICP(const Layer<TsdfVoxel> *tsdf_layer, const Pointcloud &points,
                  const Transformation &T_in, Transformation *T_out) {
  timing::Timer match_timer("icp_matching");

  PointsMatrix src;
  PointsMatrix tgt;

  matchPoints(tsdf_layer, points, T_in, &src, &tgt);

  match_timer.Stop();

  if (src.cols() <
      std::max(3, static_cast<int>(config_.subsample_keep_ratio *
                                   points.size() * config_.min_match_ratio))) {
    return false;
  }

  timing::Timer tform_timer("icp_tform");

  if (!getTransformFromMatchedPoints(src, tgt, T_out)) {
    tform_timer.Stop();
    return false;
  }
  tform_timer.Stop();

  return true;
}

bool ICP::runICP(const Layer<TsdfVoxel> *tsdf_layer, const Pointcloud &points,
                 const Transformation &T_in, Transformation *T_out) {
  if (config_.iterations == 0) {
    *T_out = T_in;
    return true;
  }

  Pointcloud subsampled_points;
  subSample(points, &subsampled_points);

  dist_info_caches_.resize(config_.num_threads);

  Transformation T_current = T_in;

  bool success = true;
  for (int i = 0; i < config_.iterations; ++i) {
    if (!stepICP(tsdf_layer, subsampled_points, T_current, T_out)) {
      *T_out = T_in;
      success = false;
      break;
    }

    T_current = *T_out;
  }

  dist_info_caches_.clear();

  return success;
}

}  // namespace voxblox
