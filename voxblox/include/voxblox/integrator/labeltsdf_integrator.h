#ifndef VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_

#include <algorithm>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

namespace voxblox {

class LabelTsdfIntegrator {
 public:
  struct Config {
    float default_truncation_distance = 0.1f;
    float max_weight = 10000.0f;
    bool voxel_carving_enabled = true;
    FloatingPoint min_ray_length_m = 0.1;
    FloatingPoint max_ray_length_m = 5.0;
    bool use_const_weight = false;
    bool allow_clear = true;
  };

  LabelTsdfIntegrator(const Config& config, LabelTsdfMap* map)
      : config_(config), map_(map), layer_(map_->getLabelTsdfLayerPtr()) {
    CHECK(map_);

    voxel_size_ = layer_->voxel_size();
    block_size_ = layer_->block_size();
    voxels_per_side_ = layer_->voxels_per_side();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
  }

  float getVoxelWeight(const Point& point_C, const Point& point_G,
                       const Point& origin, const Point& voxel_center) const {
    if (config_.use_const_weight) {
      return 1.0;
    }
    FloatingPoint dist_z = std::abs(point_C.z());
    if (dist_z > 1e-6) {
      return 1.0 / (dist_z * dist_z);
    }
    return 0.0f;
  }

  inline void readLabelPointCloud(const Transformation& T_G_C,
                                  const Pointcloud& points_C,
                                  Labels* labels) {
    CHECK_NOTNULL(labels);
    CHECK_EQ(points_C.size(), labels->size());

    std::map<Label, size_t> label_count;
    int unlabeled_count = 0;

    // Determine predominant label for segment.
    for (const auto& point_C : points_C) {
      const Point point_G = T_G_C * point_C;

      // Get the corresponding voxel by 3D position in world frame.
      Layer<LabelTsdfVoxel>::BlockType::ConstPtr block_ptr =
          layer_->getBlockPtrByCoordinates(point_G);

      if (block_ptr != nullptr) {
        const LabelTsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point_G);
        ++label_count[voxel.label];
      } else {
        ++unlabeled_count;
      }
    }

    // Find label with highest frequency.
    size_t current_max = 0u;
    Label label_max;
    for (const std::pair<Label, size_t>& label_count_pair : label_count) {
      if (label_count_pair.second > current_max) {
        label_max = label_count_pair.first;
        current_max = label_count_pair.second;
      }
    }

    if (current_max < unlabeled_count) {
      label_max = map_->get_new_label();
    }

    // Assign the dominant label to all points of segment.
    // TODO(grinvalm) can use only one label instead of vector,
    // they're all same for now
    for (size_t pt_idx = 0u; pt_idx < points_C.size(); ++pt_idx) {
      labels->at(pt_idx) = label_max;
    }
  }

  inline void updateLabelTsdfVoxel(const Point& origin,
                                   const Point& point_C,
                                   const Point& point_G,
                                   const Point& voxel_center,
                                   const Label& label,
                                   const Color& color,
                                   const float truncation_distance,
                                   const float weight,
                                   LabelTsdfVoxel* labeltsdf_voxel) {
    Eigen::Vector3d voxel_direction = point_G - voxel_center;
    Eigen::Vector3d ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0.0) {
      sdf = -sdf;
    }

    // This is for the linear drop-off in confidence behind the surface.
    float updated_weight = weight;

    const float new_weight = labeltsdf_voxel->weight + updated_weight;

    updateLabel(label, labeltsdf_voxel);

    labeltsdf_voxel->color = Color::blendTwoColors(
        labeltsdf_voxel->color, labeltsdf_voxel->weight, color, updated_weight);
    const float new_sdf =
        (sdf * updated_weight
         + labeltsdf_voxel->distance * labeltsdf_voxel->weight) / new_weight;

    labeltsdf_voxel->distance = (new_sdf > 0.0)
                               ? std::min(truncation_distance, new_sdf)
                               : std::max(-truncation_distance, new_sdf);
    labeltsdf_voxel->weight = std::min(config_.max_weight, new_weight);
  }

  void updateLabel(const Label& new_label, LabelTsdfVoxel* labeltsdf_voxel) {
    // TODO(grinvalm) add cap confidence value as in paper and consider noise.
    if (labeltsdf_voxel->label == new_label) {
      ++labeltsdf_voxel->label_confidence;
    } else {
      if (labeltsdf_voxel->label_confidence == 0.0) {
        labeltsdf_voxel->label = new_label;
      } else {
        labeltsdf_voxel->label_confidence--;
      }
    }
  }

  inline float computeDistance(const Point& origin, const Point& point_G,
                               const Point& voxel_center) {
    Eigen::Vector3d voxel_direction = point_G - voxel_center;
    Eigen::Vector3d ray_direction = point_G - origin;

    float sdf = static_cast<float>(voxel_direction.norm());
    // Figure out if it's in front of the plane or behind.
    if (voxel_direction.dot(ray_direction) < 0.0) {
      sdf = -sdf;
    }
    return sdf;
  }

  void integratePointCloud(const Transformation& T_G_C,
                           const Pointcloud& points_C,
                           const Labels& labels,
                           const Colors& colors) {
    CHECK_EQ(points_C.size(), colors.size());
    timing::Timer integrate_timer("integrate");

    const Point& origin = T_G_C.getPosition();

    for (size_t pt_idx = 0; pt_idx < points_C.size(); ++pt_idx) {
      const Point& point_C = points_C[pt_idx];
      const Point point_G = T_G_C * point_C;
      const Label& label = labels[pt_idx];
      const Color& color = colors[pt_idx];

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
      (start_scaled, end_scaled, &global_voxel_indices);
      cast_ray_timer.Stop();

      timing::Timer update_voxels_timer("integrate/update_voxels");

      BlockIndex last_block_idx = BlockIndex::Zero();
      Block<LabelTsdfVoxel>::Ptr block;

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

        if (!block || block_idx != last_block_idx) {
          block = layer_->allocateBlockPtrByIndex(block_idx);
          block->updated() = true;
          last_block_idx = block_idx;
        }

        const Point voxel_center_G =
            block->computeCoordinatesFromVoxelIndex(local_voxel_idx);
        LabelTsdfVoxel& labeltsdf_voxel =
            block->getVoxelByVoxelIndex(local_voxel_idx);

        const float weight =
            getVoxelWeight(point_C, point_G, origin, voxel_center_G);
        updateLabelTsdfVoxel(origin, point_C, point_G, voxel_center_G,
                             label, color, truncation_distance, weight,
                             &labeltsdf_voxel);
      }
      update_voxels_timer.Stop();
    }
    integrate_timer.Stop();
  }

 protected:
  Config config_;

  LabelTsdfMap* map_;
  Layer<LabelTsdfVoxel>* layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_LABELTSDF_INTEGRATOR_H_
