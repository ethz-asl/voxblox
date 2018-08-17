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

#include <lidar_align/icp.h>

ICP::ICP(Config config) : tgt_pc_ptr_(new Pointcloud), config_(config) {}

bool ICP::getTransformFromCorrelation(const Eigen::Matrix3Xf &src_demean,
                                      const Eigen::Vector3f &src_center,
                                      const Eigen::Matrix3Xf &tgt_demean,
                                      const Eigen::Vector3f &tgt_center,
                                      Transform *T_src_tgt) {
  CHECK(src_demean.cols() == tgt_demean.cols());

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = src_demean * tgt_demean.transpose();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU();
  Eigen::Matrix3f v = svd.matrixV();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0) {
    for (int x = 0; x < 3; ++x) v(x, 2) *= -1;
  }

  // Form transformation
  Eigen::Matrix3f rotation_matrix = v * u.transpose();
  Eigen::Vector3f trans = tgt_center - (rotation_matrix * src_center);

  if (!Rotation::isValidRotationMatrix(rotation_matrix)) {
    return false;
  }

  *T_src_tgt = Transform(Rotation(rotation_matrix), trans);
  return true;
}

bool ICP::getTransformFromMatchedPoints(const Eigen::Matrix3Xf &src,
                                        const Eigen::Matrix3Xf &tgt,
                                        Transform *T_src_tgt) {
  CHECK(src.cols() == tgt.cols());

  // find and remove mean
  Eigen::Vector3f src_center = src.rowwise().mean();
  Eigen::Vector3f tgt_center = tgt.rowwise().mean();

  Eigen::Matrix3Xf src_demean = src.colwise() - src_center;
  Eigen::Matrix3Xf tgt_demean = tgt.colwise() - tgt_center;

  // align
  return getTransformFromCorrelation(src_demean, src_center, tgt_demean,
                                     tgt_center, T_src_tgt);
}

void ICP::setTgtPoints(const Pointcloud &tgt_pc) {
  tgt_pc_ptr_ = boost::make_shared<Pointcloud>(*(new Pointcloud));
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(tgt_pc, *tgt_pc_ptr_, indices);
  if (tgt_pc_ptr_->size() != 0) {
    tgt_kdtree_.setInputCloud(tgt_pc_ptr_);
  }
}

void ICP::setSrcPoints(const Pointcloud &src_pc) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(src_pc, src_pc_, indices);
  src_pc_ = src_pc;
}

Pointcloud ICP::getTransformedSrcPoints(void) const {
  Pointcloud tformed_pc;
  pcl::transformPointCloud(src_pc_, tformed_pc,
                           T_src_tgt_.getTransformationMatrix());

  return tformed_pc;
}

Transform ICP::getCurrentTransform() const { return T_src_tgt_; }

void ICP::setCurrentTransform(Transform T_src_tgt) { T_src_tgt_ = T_src_tgt; }

void ICP::matchPoints(Eigen::Matrix3Xf *src, Eigen::Matrix3Xf *tgt) const {
  size_t num_match;
  if (config_.match_second_closest) {
    num_match = 2;
  } else {
    num_match = 1;
  }

  std::vector<int> kdtree_idx(num_match);
  std::vector<float> kdtree_dist(num_match);

  Pointcloud tformed_pc;
  Eigen::Matrix4f tform = T_src_tgt_.getTransformationMatrix();
  pcl::transformPointCloud(src_pc_, tformed_pc, tform);

  // build alignment matrices
  for (size_t i = 0; i < src_pc_.size(); ++i) {
    tgt_kdtree_.nearestKSearch(tformed_pc[i], num_match, kdtree_idx,
                               kdtree_dist);

    Point matching_point = tgt_pc_ptr_->at(kdtree_idx.back());

    src->col(i) = Eigen::Vector3f(src_pc_[i].x, src_pc_[i].y, src_pc_[i].z);
    tgt->col(i) =
        Eigen::Vector3f(matching_point.x, matching_point.y, matching_point.z);
  }
}

void ICP::removeOutliers(const Eigen::Matrix3Xf &src,
                         const Eigen::Matrix3Xf &tgt,
                         const float max_inlier_dist,
                         Eigen::Matrix3Xf *src_filt,
                         Eigen::Matrix3Xf *tgt_filt) {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      src_vec;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      tgt_vec;

  for (size_t i = 0; i < src.cols(); ++i) {
    if ((src.col(i) - tgt.col(i)).squaredNorm() <
        max_inlier_dist * max_inlier_dist) {
      src_vec.push_back(src.col(i));
      tgt_vec.push_back(tgt.col(i));
    }
  }

  src_filt->resize(src.rows(), src_vec.size());
  tgt_filt->resize(tgt.rows(), tgt_vec.size());

  for (size_t i = 0; i < src_vec.size(); ++i) {
    src_filt->col(i) = src_vec[i];
    tgt_filt->col(i) = tgt_vec[i];
  }
}

bool ICP::getFilteredMatchingPoints(Eigen::Matrix3Xf *src_filt,
                                    Eigen::Matrix3Xf *tgt_filt) const {
  const size_t npts = src_pc_.size();

  Eigen::Matrix3Xf src(3, npts);
  Eigen::Matrix3Xf tgt(3, npts);

  if ((tgt_pc_ptr_->size() == 0) || (src_pc_.size() == 0)) {
    return false;
  }

  matchPoints(&src, &tgt);

  removeOutliers(src, tgt, config_.max_inlier_dist, src_filt, tgt_filt);

  // for(size_t i = 0; i < src_filt->cols(); ++i){
  //  std::cout << src_filt->col(i).transpose() << " " <<
  //  tgt_filt->col(i).transpose() << " " << ((src_filt->col(i) -
  //  tgt_filt->col(i)).squaredNorm()) << std::endl;
  //}
}

bool ICP::stepICP() {
  Eigen::Matrix3Xf src_filt;
  Eigen::Matrix3Xf tgt_filt;

  if (!getFilteredMatchingPoints(&src_filt, &tgt_filt) ||
      !getTransformFromMatchedPoints(src_filt, tgt_filt, &T_src_tgt_)) {
    return false;
  }

  return true;
}

bool ICP::runICP() {
  if ((tgt_pc_ptr_->size() == 0) || (src_pc_.size() == 0)) {
    return false;
  }
  for (size_t i = 0; i < config_.iterations; ++i) {
    if (!stepICP()) {
      return false;
    }
  }
  return true;
}