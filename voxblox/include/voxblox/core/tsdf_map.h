#ifndef VOXBLOX_CORE_TSDF_MAP_H_
#define VOXBLOX_CORE_TSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

class TsdfMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint tsdf_voxel_size = 0.2;
    size_t tsdf_voxels_per_side = 16u;
  };

  explicit TsdfMap(const Config& config)
      : tsdf_layer_(new Layer<TsdfVoxel>(config.tsdf_voxel_size,
                                         config.tsdf_voxels_per_side)) {
    block_size_ = config.tsdf_voxel_size * config.tsdf_voxels_per_side;
  }

  // NOTE(mereweth@jpl.nasa.gov) - for convenience with Python bindings
  TsdfMap(Layer<TsdfVoxel>::Ptr tsdf_layer)
     : tsdf_layer_(tsdf_layer) {
    if (!tsdf_layer) {
      // TODO(mereweth@jpl.nasa.gov) - throw std exception for Python to catch?
      throw std::runtime_error(std::string("Null Layer<TsdfVoxel>::Ptr") +
                               " in TsdfMap constructor");
    }
    block_size_ = tsdf_layer_->block_size();
  }
    
  // Creates a new TsdfMap based on a COPY of this layer.
  explicit TsdfMap(const Layer<TsdfVoxel>& layer)
      : TsdfMap(aligned_shared<Layer<TsdfVoxel>>(layer)) {}

  // Creates a new TsdfMap that contains this layer.
  explicit TsdfMap(Layer<TsdfVoxel>::Ptr layer) : tsdf_layer_(layer) {
    CHECK(layer);
    block_size_ = layer->block_size();
  }

  virtual ~TsdfMap() {}

  Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }
  const Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return tsdf_layer_->voxel_size(); }

  // EigenDRef is fully dynamic stride type alias for Numpy array slices
  // Use column-major matrices; column-by-column traversal is faster

  // convenience alias borrowed from pybind11
  using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  template <typename MatrixType> using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

  unsigned int coordPlaneSliceGetDistanceWeight(
    unsigned int free_plane_index,
    double free_plane_val,
    EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorXd> distances,
    Eigen::Ref<Eigen::VectorXd> weights,
    unsigned int max_points = 100000) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_MAP_H_
