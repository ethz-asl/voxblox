#ifndef VOXBLOX_CORE_TSDF_MAP_H_
#define VOXBLOX_CORE_TSDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <string>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {
/**
 * Map holding a Truncated Signed Distance Field Layer. Contains functions for
 * interacting with the layer and getting gradient and distance information.
 */
class TsdfMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TsdfMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint tsdf_voxel_size = 0.2;
    size_t tsdf_voxels_per_side = 16u;

    std::string print() const;
  };

  explicit TsdfMap(const Config& config)
      : tsdf_layer_(new Layer<TsdfVoxel>(config.tsdf_voxel_size,
                                         config.tsdf_voxels_per_side)) {
    block_size_ = config.tsdf_voxel_size * config.tsdf_voxels_per_side;
  }

  /// Creates a new TsdfMap based on a COPY of this layer.
  explicit TsdfMap(const Layer<TsdfVoxel>& layer)
      : TsdfMap(aligned_shared<Layer<TsdfVoxel>>(layer)) {}

  /// Creates a new TsdfMap that contains this layer.
  explicit TsdfMap(Layer<TsdfVoxel>::Ptr layer) : tsdf_layer_(layer) {
    if (!layer) {
      /* NOTE(mereweth@jpl.nasa.gov) - throw std exception for Python to catch
       * This is idiomatic when wrapping C++ code for Python, especially with
       * pybind11
       */
      throw std::runtime_error(std::string("Null Layer<TsdfVoxel>::Ptr") +
                               " in TsdfMap constructor");
    }

    CHECK(layer);
    block_size_ = layer->block_size();
  }

  virtual ~TsdfMap() {}

  Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }
  const Layer<TsdfVoxel>& getTsdfLayer() const { return *tsdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return tsdf_layer_->voxel_size(); }

  /* NOTE(mereweth@jpl.nasa.gov)
   * EigenDRef is fully dynamic stride type alias for Numpy array slices
   * Use column-major matrices; column-by-column traversal is faster
   * Convenience alias borrowed from pybind11
   */
  using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  template <typename MatrixType>
  using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

  /**
   *  Extract all voxels on a slice plane that is parallel to one of the
   * axis-aligned planes. free_plane_index specifies the free coordinate
   * (zero-based; x, y, z order) free_plane_val specifies the plane intercept
   * coordinate along that axis
   */
  unsigned int coordPlaneSliceGetDistanceWeight(
      unsigned int free_plane_index, double free_plane_val,
      EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXd> distances,
      Eigen::Ref<Eigen::VectorXd> weights, unsigned int max_points) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_TSDF_MAP_H_
