#ifndef VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_

#include <algorithm>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/labeltsdf_map.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class LabelTsdfIntegrator : public TsdfIntegrator {
 public:
  LabelTsdfIntegrator(const Config& config,
                      Layer<TsdfVoxel>* tsdf_layer,
                      Layer<LabelVoxel>* label_layer,
                      Label* highest_label)
      : TsdfIntegrator(config, CHECK_NOTNULL(tsdf_layer)),
        label_layer_(CHECK_NOTNULL(label_layer)),
        highest_label_(CHECK_NOTNULL(highest_label)) {
    CHECK(label_layer_);
  }

  Label get_fresh_label() {
    return ++(*highest_label_);
  }

  inline void computePointCloudLabel(const Transformation& T_G_C,
                                     const Pointcloud& points_C,
                                     Labels* labels) {
    CHECK_NOTNULL(labels);
    labels->resize(points_C.size());

    std::map<Label, size_t> label_count;
    size_t unlabeled_count = 0u;

    // Determine predominant label for segment.
    for (const Point& point_C : points_C) {
      const Point point_G = T_G_C * point_C;

      // Get the corresponding voxel by 3D position in world frame.
      Layer<LabelVoxel>::BlockType::ConstPtr block_ptr =
          label_layer_->getBlockPtrByCoordinates(point_G);

      if (block_ptr != nullptr) {
        const LabelVoxel& voxel = block_ptr->getVoxelByCoordinates(point_G);
        ++label_count[voxel.label];
      } else {
        ++unlabeled_count;
      }
    }

    // Find label with highest frequency.
    // TODO(grinvalm) add threshold, probably in the form of a ratio
    size_t current_max = 0u;
    Label label_max;
    for (const std::pair<Label, size_t>& label_count_pair : label_count) {
      if (label_count_pair.second > current_max) {
        label_max = label_count_pair.first;
        current_max = label_count_pair.second;
      }
    }

    if (current_max < unlabeled_count) {
      label_max = get_fresh_label();
    }

    // Assign the dominant label to all points of segment.
    // TODO(grinvalm) can use only one label instead of vector,
    // they're all same for now
    for (size_t point_idx = 0u; point_idx < points_C.size(); ++point_idx) {
      labels->at(point_idx) = label_max;
    }
  }

  inline void updateLabelVoxel(const Label& label,
                               LabelVoxel* label_voxel) {
    CHECK_NOTNULL(label_voxel);

    // TODO(grinvalm) use cap confidence value as in paper and consider noise.
    if (label_voxel->label == label) {
      ++label_voxel->label_confidence;
    } else {
      if (label_voxel->label_confidence <= 0.0f) {
        label_voxel->label = label;
        if (*highest_label_ < label) {
          *highest_label_ = label;
        }
      } else {
        label_voxel->label_confidence--;
      }
    }
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C,
                           const Colors& colors,
                           const Labels& labels) {
    CHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    for (size_t point_idx = 0u; point_idx < points_C.size(); ++point_idx) {
      const Point& point_C = points_C[point_idx];
      const Point point_G = T_G_C * point_C;
      const Label& label = labels[point_idx];
      const Color& color = colors[point_idx];

      FloatingPoint ray_distance = (point_G - origin).norm();
      if (ray_distance < config_.min_ray_length_m) {
        continue;
      } else if (ray_distance > config_.max_ray_length_m) {
        // TODO(helenol): clear until max ray length instead.
        continue;
      }

      FloatingPoint truncation_distance = config_.default_truncation_distance;

      const Ray unit_ray = (point_G - origin).normalized();

      const Point ray_end = point_G + unit_ray * truncation_distance;
      const Point ray_start = config_.voxel_carving_enabled
                                  ? origin
                                  : (point_G - unit_ray * truncation_distance);

      const Point start_scaled = ray_start * voxel_size_inv_;
      const Point end_scaled = ray_end * voxel_size_inv_;

      IndexVector global_voxel_indices;
      timing::Timer cast_ray_timer("integrate/cast_ray");
      castRay(start_scaled, end_scaled, &global_voxel_indices);
      cast_ray_timer.Stop();

      timing::Timer update_voxels_timer("integrate/update_voxels");

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<TsdfVoxel>::Ptr tsdf_block;
      Block<LabelVoxel>::Ptr label_block;

      for (const AnyIndex& global_voxel_idx : global_voxel_indices) {
        BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_inv_);
        VoxelIndex local_voxel_idx =
            getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        if (local_voxel_idx.x() < 0) {
          local_voxel_idx.x() += voxels_per_side_;
        }
        if (local_voxel_idx.y() < 0) {
          local_voxel_idx.y() += voxels_per_side_;
        }
        if (local_voxel_idx.z() < 0) {
          local_voxel_idx.z() += voxels_per_side_;
        }

        if (!tsdf_block || block_idx != last_block_idx) {
          if (!tsdf_block != !label_block) {
            LOG(FATAL) << "Block allocation differs between the two layers.";
          }
          tsdf_block = layer_->allocateBlockPtrByIndex(block_idx);
          label_block = label_layer_->allocateBlockPtrByIndex(block_idx);

          tsdf_block->updated() = true;
          label_block->updated() = true;

          last_block_idx = block_idx;
        }

        CHECK(tsdf_block);
        CHECK(label_block);

        const Point voxel_center_G =
            tsdf_block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        TsdfVoxel& tsdf_voxel =
            tsdf_block->getVoxelByVoxelIndex(local_voxel_idx);
        LabelVoxel& label_voxel =
            label_block->getVoxelByVoxelIndex(local_voxel_idx);

        const float weight =
            getVoxelWeight(point_C, point_G, origin, voxel_center_G);
        updateTsdfVoxel(origin, point_C, point_G, voxel_center_G, color,
                        truncation_distance, weight, &tsdf_voxel);

        updateLabelVoxel(label, &label_voxel);
      }
      update_voxels_timer.Stop();
    }
    integrate_timer.Stop();
  }

 protected:
  Layer<LabelVoxel>* label_layer_;

  Label* highest_label_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_
