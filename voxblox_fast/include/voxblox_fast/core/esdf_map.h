#ifndef VOXBLOX_FAST_CORE_ESDF_MAP_H_
#define VOXBLOX_FAST_CORE_ESDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox_fast/core/common.h"
#include "voxblox_fast/core/layer.h"
#include "voxblox_fast/core/voxel.h"
#include "voxblox_fast/interpolator/interpolator.h"

namespace voxblox_fast {

class EsdfMap {
 public:
  typedef std::shared_ptr<EsdfMap> Ptr;

  struct Config {
    FloatingPoint esdf_voxel_size = 0.2;
    size_t esdf_voxels_per_side = 16u;
  };

  explicit EsdfMap(const Config& config)
      : esdf_layer_(new Layer<EsdfVoxel>(config.esdf_voxel_size,
                                         config.esdf_voxels_per_side)),
        interpolator_(esdf_layer_.get()) {
    block_size_ = config.esdf_voxel_size * config.esdf_voxels_per_side;
  }

  virtual ~EsdfMap() {}

  Layer<EsdfVoxel>* getEsdfLayerPtr() { return esdf_layer_.get(); }
  const Layer<EsdfVoxel>& getEsdfLayer() const { return *esdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return esdf_layer_->voxel_size(); }

  // Specific accessor functions for esdf maps.
  // Returns true if the point exists in the map AND is observed.
  bool getDistanceAtPosition(const Eigen::Vector3d& position,
                             double* distance) const;

  bool getDistanceAndGradientAtPosition(const Eigen::Vector3d& position,
                                        double* distance,
                                        Eigen::Vector3d* gradient) const;

  bool isObserved(const Eigen::Vector3d& position) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<EsdfVoxel>::Ptr esdf_layer_;

  // Interpolator for the layer.
  Interpolator<EsdfVoxel> interpolator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_FAST_CORE_ESDF_MAP_H_
