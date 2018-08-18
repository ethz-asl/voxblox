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

#include "voxblox/alignment/icp.h"
#include "voxblox/interpolator/interpolator.h"

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
                      const AlignedVector<Pointcloud> &points,
                      const Transformation &T, PointsMatrix *src,
                      PointsMatrix *tgt) const {
  AlignedVector<std::shared_ptr<ThreadSafeIndex>> index_getters;
  for (const Pointcloud &pointcloud : points) {
    index_getters.push_back(
        std::make_shared<ThreadSafeIndex>(pointcloud.size()));
  }

  std::list<std::thread> threads;
  AlignedVector<Pointcloud> src_points(config_.num_threads);
  AlignedVector<Pointcloud> tgt_points(config_.num_threads);

  for (size_t i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(&ICP::calcMatches, this, tsdf_layer, points,
                         &index_getters, T, &src_points[i], &tgt_points[i]);
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  size_t a_size = 0;
  for (const Pointcloud &pointcloud : points) {
    a_size += pointcloud.size();
  }

  size_t total_size = 0;
  for (const Pointcloud &pointcloud : src_points) {
    total_size += pointcloud.size();
  }

  src->resize(3, total_size);
  tgt->resize(3, total_size);

  size_t index = 0;
  for (size_t i = 0; i < src_points.size(); ++i) {
    for (size_t j = 0; j < src_points[i].size(); ++j) {
      src->col(index) = src_points[i][j];
      tgt->col(index) = tgt_points[i][j];
      ++index;
    }
  }
}

void ICP::calcMatches(
    const Layer<TsdfVoxel> *tsdf_layer, const AlignedVector<Pointcloud> &points,
    AlignedVector<std::shared_ptr<ThreadSafeIndex>> *index_getters,
    const Transformation &T, Pointcloud *src, Pointcloud *tgt) const {
  Interpolator<TsdfVoxel> interpolator(tsdf_layer);

  constexpr bool kInterpolateDist = true;
  constexpr bool kInterpolateGrad = false;
  constexpr FloatingPoint kMinGradMag = 0.1;

  for (size_t i = 0; i < points.size(); ++i) {
    size_t point_idx;
    while (index_getters->at(i)->getNextIndex(&point_idx)) {
      const Point &point = points[i][point_idx];
      Point point_tformed = T * point;

      Point grad;
      FloatingPoint distance;

      if (interpolator.getDistance(point_tformed, &distance,
                                   kInterpolateDist) &&
          interpolator.getGradient(point_tformed, &grad, kInterpolateGrad) &&
          (grad.squaredNorm() > kMinGradMag)) {
        src->push_back(point);
        tgt->push_back(point_tformed - distance * grad.normalized());
      }
    }
  }
}

bool ICP::stepICP(const Layer<TsdfVoxel> *tsdf_layer,
                  const AlignedVector<Pointcloud> &points,
                  const Transformation &T_in, Transformation *T_out) {
  PointsMatrix src;
  PointsMatrix tgt;

  size_t total_size = 0;
  for (const Pointcloud &pointcloud : points) {
    total_size += pointcloud.size();
  }

  matchPoints(tsdf_layer, points, T_in, &src, &tgt);

  if (src.cols() <
      std::max(3, static_cast<int>(total_size * config_.min_match_ratio))) {
    return false;
  }

  if (!getTransformFromMatchedPoints(src, tgt, T_out)) {
    return false;
  }

  return true;
}

void ICP::downsampleCloud(const Pointcloud &points,
                          ThreadSafeIndex *index_getter,
                          Pointcloud *points_downsampled) {
  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point &point = points[point_idx];
    const GlobalIndex voxel_idx =
        getGridIndexFromPoint<GlobalIndex>(point, config_.voxel_size_inv);

    if (voxel_approx_set_.replaceHash(voxel_idx)) {
      points_downsampled->push_back(point);
    }
  }
}

bool ICP::runICP(const Layer<TsdfVoxel> *tsdf_layer, const Pointcloud &points,
                 const Transformation &T_in, Transformation *T_out) {
  if (config_.iterations == 0) {
    *T_out = T_in;
    return true;
  }

  voxel_approx_set_.resetApproxSet();

  ThreadSafeIndex index_getter(points.size());
  std::list<std::thread> threads;
  AlignedVector<Pointcloud> points_downsampled(config_.num_threads);
  for (size_t i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(&ICP::downsampleCloud, this, points, &index_getter,
                         &points_downsampled[i]);
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  Transformation T_current = T_in;

  for (int i = 0; i < config_.iterations; ++i) {
    if (!stepICP(tsdf_layer, points_downsampled, T_current, T_out)) {
      *T_out = T_in;
      return false;
    }

    T_current = *T_out;
  }
  return true;
}

}  // namespace voxblox
