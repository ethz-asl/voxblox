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

  bool success;
  Rotation rotation;
  if (refine_roll_pitch) {
    success =
        getRotationFromMatchedPoints<3>(src_demean, tgt_demean, &rotation);
  } else {
    success =
        getRotationFromMatchedPoints<2>(src_demean, tgt_demean, &rotation);
  }

  const Point trans = tgt_center - rotation.rotate(src_center);

  *T = Transformation(rotation, trans);
  return success;
}

void ICP::addNormalizedPointInfo(const Point& point_gradient,
                                 SquareMatrix<6>* info_mat) {
  // find othogonal vectors
  const Point point_norm = point_gradient.normalized();
  Point oth_vec_a, oth_vec_b;

  // avoid singularity by choosing convient up direction
  if (point_norm(2) < 0.5) {
    const Point up(0.0, 0.0, 1.0);
    oth_vec_a = up.cross(point_norm).normalized();
  } else {
    const Point up(0.0, 1.0, 0.0);
    oth_vec_a = up.cross(point_norm).normalized();
  }
  oth_vec_b = point_norm.cross(oth_vec_a).normalized();

  // add translational point information
  info_mat->topLeftCorner<3, 3>() += point_norm * point_norm.transpose();
  // add rotational point information
  info_mat->bottomRightCorner<3, 3>() +=
      oth_vec_a * oth_vec_a.transpose() + oth_vec_b * oth_vec_b.transpose();
}

void ICP::matchPoints(const Pointcloud& points, const Transformation& T,
                      PointsMatrix* src, PointsMatrix* tgt,
                      SquareMatrix<6>* info_mat) {
  constexpr bool kInterpolateDist = false;
  constexpr bool kInterpolateGrad = false;
  constexpr FloatingPoint kMinGradMag = 0.1;

  src->resize(3, points.size());
  tgt->resize(3, points.size());

  // epsilion to ensure the matrix can be inverted
  *info_mat = kEpsilon * SquareMatrix<6>::Identity();

  size_t idx = 0;
  for (const Point& point : points) {
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

      addNormalizedPointInfo(gradient, info_mat);

      // uninterpolated distance is to the center of the voxel, as we now have
      // the gradient we can use it to do better
      const Point voxel_center =
          getCenterPointFromGridIndex<AnyIndex>(voxel_idx, voxel_size_);
      distance += gradient.dot(point_tformed - voxel_center);

      src->col(idx) = point_tformed;
      tgt->col(idx) = point_tformed - distance * gradient;
      ++idx;
    }
  }

  // std::cout << "info mat\n" << info_mat << std::endl;

  src->conservativeResize(3, idx);
  tgt->conservativeResize(3, idx);
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

bool ICP::stepICP(const Pointcloud& points, const Transformation& T_in,
                  Transformation* T_delta, SquareMatrix<6>* info_mat) {
  PointsMatrix src;
  PointsMatrix tgt;

  matchPoints(points, T_in, &src, &tgt, info_mat);

  if (src.cols() <
      std::max(3, static_cast<int>(points.size() * config_.min_match_ratio))) {
    return false;
  }

  if (!getTransformFromMatchedPoints(src, tgt, config_.refine_roll_pitch,
                                     T_delta)) {
    return false;
  }

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

  SquareMatrix<6> base_info_mat;
  base_info_mat.topLeftCorner<3,3> = config_.inital_translation_weighting * SquareMatrix<3>::Identity();
  base_info_mat.bottomRightCorner<3,3> = config_.inital_rotation_weighting * SquareMatrix<3>::Identity();

  SquareMatrix<6> est_info_mat;

  bool success = true;
  Transformation T_delta;
  for (int i = 0; i < config_.iterations; ++i) {
    if (!stepICP(subsampled_points, T_current, &T_delta, &est_info_mat)) {
      *T_out = T_in;
      success = false;
      break;
    }

    T_current = T_delta * T_current;
  }

  interpolator_.reset();

  T_delta = T_in.inverse() * T_current;

  //todo don't assume all axes independent
  const Transformation::Vector6 weight =
      est_info_mat.diagonal().array() /
      (base_info_mat.diagonal() + est_info_mat.diagonal()).array();
  T_delta = Transformation::exp(T_delta.log().array() * weight.array());

  *T_out = T_in * T_delta;

  return success;
}

}  // namespace voxblox
