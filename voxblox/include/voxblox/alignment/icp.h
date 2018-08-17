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
#ifndef VOXBLOX_INTEGRATOR_ICP_H_
#define VOXBLOX_INTEGRATOR_ICP_H_

#include <algorithm>

class ICP {
 public:
  struct Config {
    Config() { iterations = 20; }
    size_t iterations;
  };

  ICP(Config config);

  void setTgtPoints(const Pointcloud &tgt_pc);

  void setSrcPoints(const Pointcloud &src_pc);

  Pointcloud getTransformedSrcPoints() const;

  Transform getCurrentTransform() const;

  void setCurrentTransform(Transform T_src_tgt);

  bool runICP();

  bool getFilteredMatchingPoints(Eigen::Matrix3Xf *src_filt,
                                 Eigen::Matrix3Xf *tgt_filt) const;

  static bool getTransformFromMatchedPoints(const Eigen::Matrix3Xf &src,
                                            const Eigen::Matrix3Xf &tgt,
                                            Transform *T_src_tgt);

 private:
  Transform T_src_tgt_;

  static bool getTransformFromCorrelation(const Eigen::Matrix3Xf &src_demean,
                                          const Eigen::Vector3f &src_center,
                                          const Eigen::Matrix3Xf &tgt_demean,
                                          const Eigen::Vector3f &tgt_center,
                                          Transform *T_src_tgt);

  void matchPoints(Eigen::Matrix3Xf *src, Eigen::Matrix3Xf *tgt) const;

  static void removeOutliers(const Eigen::Matrix3Xf &src,
                             const Eigen::Matrix3Xf &tgt,
                             const float max_inlier_dist,
                             Eigen::Matrix3Xf *src_filt,
                             Eigen::Matrix3Xf *tgt_filt);

  bool stepICP();

  Pointcloud src_pc_;
  Pointcloud::Ptr tgt_pc_ptr_;
  pcl::KdTreeFLANN<Point> tgt_kdtree_;
  Config config_;
};

#endif