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

#include <random>

namespace voxblox {

ICP::ICP(const Config& config) : config_(config) {}

bool ICP::getTransformFromMatchedPoints(const PointsMatrix& src,
                                        const PointsMatrix& tgt,
                                        const bool refine_roll_pitch,
                                        Transformation* T_tgt_src) {
  CHECK(src.cols() == tgt.cols());
  CHECK_NOTNULL(T_tgt_src);

  // find and remove mean
  const Point src_center = src.rowwise().mean();
  const Point tgt_center = tgt.rowwise().mean();

  const PointsMatrix src_demean = src.colwise() - src_center;
  const PointsMatrix tgt_demean = tgt.colwise() - tgt_center;

  bool success;
  Rotation R_tgt_src;
  if (refine_roll_pitch) {
    success =
        getRotationFromMatchedPoints<3>(src_demean, tgt_demean, &R_tgt_src);
  } else {
    success =
        getRotationFromMatchedPoints<2>(src_demean, tgt_demean, &R_tgt_src);
  }

  const Point t_tgt_src = tgt_center - R_tgt_src.rotate(src_center);

  *T_tgt_src = Transformation(R_tgt_src, t_tgt_src);
  return success;
}

void ICP::addNormalizedPointInfo(const Point& point,
                                 const Point& normalized_point_normal,
                                 Vector6* info_vector) {
  DCHECK(info_vector != nullptr);

  // add translational point information
  info_vector->head<3>() +=
      2.0 * normalized_point_normal.cwiseProduct(normalized_point_normal);
  // add rotational point information (todo verify correct)
  info_vector->tail<3>() +=
      2.0 * Point(point.y() * point.y() * normalized_point_normal.z() *
                          normalized_point_normal.z() +
                      point.z() * point.z() * normalized_point_normal.y() *
                          normalized_point_normal.y(),
                  point.x() * point.x() * normalized_point_normal.z() *
                          normalized_point_normal.z() +
                      point.z() * point.z() * normalized_point_normal.x() *
                          normalized_point_normal.x(),
                  point.x() * point.x() * normalized_point_normal.y() *
                          normalized_point_normal.y() +
                      point.y() * point.y() * normalized_point_normal.x() *
                          normalized_point_normal.x());
}

void ICP::matchPoints(const Pointcloud& points, const size_t start_idx,
                      const Transformation& T_tsdf_sensor, PointsMatrix* src,
                      PointsMatrix* tgt, Vector6* info_vector) {
  CHECK_NOTNULL(src);
  CHECK_NOTNULL(tgt);
  CHECK_NOTNULL(info_vector);

  constexpr bool kInterpolateDist = false;
  constexpr bool kInterpolateGrad = false;
  constexpr FloatingPoint kMinGradMag = 0.1;

  src->resize(3, config_.mini_batch_size);
  tgt->resize(3, config_.mini_batch_size);

  // epsilon to ensure we can always divide by it
  info_vector->setConstant(kEpsilon);

  size_t idx = 0;
  for (size_t i = start_idx;
       i < std::min(points.size(), start_idx + config_.mini_batch_size); ++i) {
    const Point& point_sensor = points[i];
    const Point point_tsdf = T_tsdf_sensor * point_sensor;
    const AnyIndex voxel_idx =
        getGridIndexFromPoint<AnyIndex>(point_tsdf, voxel_size_inv_);

    FloatingPoint distance;
    Point gradient;

    if (interpolator_->getDistance(point_tsdf, &distance, kInterpolateDist) &&
        interpolator_->getGradient(point_tsdf, &gradient, kInterpolateGrad) &&
        (gradient.squaredNorm() > kMinGradMag)) {
      gradient.normalize();

      addNormalizedPointInfo(point_tsdf - T_tsdf_sensor.getPosition(), gradient,
                             info_vector);

      // uninterpolated distance is to the center of the voxel, as we now have
      // the gradient we can use it to do better
      const Point voxel_center =
          getCenterPointFromGridIndex<AnyIndex>(voxel_idx, voxel_size_);
      distance += gradient.dot(point_tsdf - voxel_center);

      src->col(idx) = point_tsdf;
      tgt->col(idx) = point_tsdf - distance * gradient;
      ++idx;
    }
  }

  src->conservativeResize(3, idx);
  tgt->conservativeResize(3, idx);
}

bool ICP::stepICP(const Pointcloud& points, const size_t start_idx,
                  const Transformation& inital_T_tsdf_sensor,
                  Transformation* delta_T_tsdf_sensor, Vector6* info_vector) {
  CHECK_NOTNULL(delta_T_tsdf_sensor);
  CHECK_NOTNULL(info_vector);

  PointsMatrix src;
  PointsMatrix tgt;

  matchPoints(points, start_idx, inital_T_tsdf_sensor, &src, &tgt, info_vector);

  if (src.cols() < std::max(3, static_cast<int>(config_.mini_batch_size *
                                                config_.min_match_ratio))) {
    return false;
  }

  if (!getTransformFromMatchedPoints(src, tgt, config_.refine_roll_pitch,
                                     delta_T_tsdf_sensor)) {
    return false;
  }

  return true;
}

void ICP::runThread(const Pointcloud& points,
                    Transformation* current_T_tsdf_sensor,
                    Vector6* base_info_vector, size_t* num_updates) {
  CHECK_NOTNULL(current_T_tsdf_sensor);
  CHECK_NOTNULL(base_info_vector);
  CHECK_NOTNULL(num_updates);

  Transformation current_thread_T_tsdf_sensor;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_thread_T_tsdf_sensor = *current_T_tsdf_sensor;
  }

  while (true) {
    const size_t start_idx = atomic_idx_.fetch_add(config_.mini_batch_size);
    if (start_idx > config_.subsample_keep_ratio * points.size()) {
      break;
    }

    Vector6 est_info_vector;
    Transformation delta_T_tsdf_sensor;

    if (stepICP(points, start_idx, current_thread_T_tsdf_sensor,
                &delta_T_tsdf_sensor, &est_info_vector)) {
      std::lock_guard<std::mutex> lock(mutex_);

      // flip tform order
      Transformation T_temp = delta_T_tsdf_sensor * *current_T_tsdf_sensor;
      delta_T_tsdf_sensor = current_T_tsdf_sensor->inverse() * T_temp;

      const Vector6 weight = est_info_vector.array() /
                             (*base_info_vector + est_info_vector).array();
      delta_T_tsdf_sensor =
          Transformation::exp(weight.cwiseProduct(delta_T_tsdf_sensor.log()));

      *base_info_vector += est_info_vector;

      *current_T_tsdf_sensor = *current_T_tsdf_sensor * delta_T_tsdf_sensor;
      current_thread_T_tsdf_sensor = *current_T_tsdf_sensor;
      ++(*num_updates);
    }
  }
}

size_t ICP::runICP(const Layer<TsdfVoxel>& tsdf_layer, const Pointcloud& points,
                   const Transformation& inital_T_tsdf_sensor,
                   Transformation* refined_T_tsdf_sensor, const unsigned seed) {
  CHECK_NOTNULL(refined_T_tsdf_sensor);

  interpolator_ = std::make_shared<Interpolator<TsdfVoxel>>(&tsdf_layer);
  voxel_size_ = tsdf_layer.voxel_size();
  voxel_size_inv_ = tsdf_layer.voxel_size_inv();

  Pointcloud shuffled_points = points;

  // randomize point order
  std::shuffle(shuffled_points.begin(), shuffled_points.end(),
               std::default_random_engine(seed));

  *refined_T_tsdf_sensor = inital_T_tsdf_sensor;

  Vector6 base_info_vector;
  base_info_vector.head<3>().setConstant(config_.inital_translation_weighting);
  base_info_vector.tail<3>().setConstant(config_.inital_rotation_weighting);

  std::list<std::thread> threads;
  atomic_idx_.store(0);
  size_t num_updates = 0;

  for (size_t i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(&ICP::runThread, this, shuffled_points,
                         refined_T_tsdf_sensor, &base_info_vector,
                         &num_updates);
  }
  for (std::thread& thread : threads) {
    thread.join();
  }

  interpolator_.reset();

  return num_updates;
}

}  // namespace voxblox
