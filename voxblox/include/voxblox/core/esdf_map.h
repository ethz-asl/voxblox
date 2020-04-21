#ifndef VOXBLOX_CORE_ESDF_MAP_H_
#define VOXBLOX_CORE_ESDF_MAP_H_

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/interpolator/interpolator.h"
#include "voxblox/io/layer_io.h"

namespace voxblox {
/**
 * Map holding a Euclidean Signed Distance Field Layer. Contains functions for
 * interacting with the layer and getting gradient and distance information.
 */
class EsdfMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<EsdfMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint esdf_voxel_size = 0.2;
    size_t esdf_voxels_per_side = 16u;
  };

  explicit EsdfMap(const Config& config)
      : esdf_layer_(new Layer<EsdfVoxel>(config.esdf_voxel_size,
                                         config.esdf_voxels_per_side)),
        interpolator_(esdf_layer_.get()) {
    block_size_ = config.esdf_voxel_size * config.esdf_voxels_per_side;
  }

  /// Creates a new EsdfMap based on a COPY of this layer.
  explicit EsdfMap(const Layer<EsdfVoxel>& layer)
      : EsdfMap(aligned_shared<Layer<EsdfVoxel>>(layer)) {}

  /// Creates a new EsdfMap that contains this layer.
  explicit EsdfMap(Layer<EsdfVoxel>::Ptr layer)
      : esdf_layer_(layer), interpolator_(CHECK_NOTNULL(esdf_layer_.get())) {
    block_size_ = layer->block_size();
  }

  virtual ~EsdfMap() {}

  Layer<EsdfVoxel>* getEsdfLayerPtr() { return esdf_layer_.get(); }
  const Layer<EsdfVoxel>& getEsdfLayer() const { return *esdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return esdf_layer_->voxel_size(); }

  /**
   * Specific accessor functions for esdf maps.
   * Returns true if the point exists in the map AND is observed.
   * These accessors use Vector3d and doubles explicitly rather than
   * FloatingPoint to have a standard, cast-free interface to planning
   * functions.
   */
  bool getDistanceAtPosition(const Eigen::Vector3d& position,
                             double* distance) const;
  bool getDistanceAtPosition(const Eigen::Vector3d& position, bool interpolate,
                             double* distance) const;

  bool getDistanceAndGradientAtPosition(const Eigen::Vector3d& position,
                                        double* distance,
                                        Eigen::Vector3d* gradient) const;
  bool getDistanceAndGradientAtPosition(const Eigen::Vector3d& position,
                                        bool interpolate, double* distance,
                                        Eigen::Vector3d* gradient) const;

  bool isObserved(const Eigen::Vector3d& position) const;

  // NOTE(mereweth@jpl.nasa.gov)
  // EigenDRef is fully dynamic stride type alias for Numpy array slices
  // Use column-major matrices; column-by-column traversal is faster
  // Convenience alias borrowed from pybind11
  using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  template <typename MatrixType>
  using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

  // Convenience functions for querying many points at once from Python
  void batchGetDistanceAtPosition(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXd> distances,
      Eigen::Ref<Eigen::VectorXi> observed) const;

  void batchGetDistanceAndGradientAtPosition(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXd> distances,
      EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& gradients,
      Eigen::Ref<Eigen::VectorXi> observed) const;

  void batchIsObserved(
      EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXi> observed) const;

  unsigned int coordPlaneSliceGetCount(unsigned int free_plane_index,
                                       double free_plane_val) const;

  /**
   * Extract all voxels on a slice plane that is parallel to one of the
   * axis-aligned planes. free_plane_index specifies the free coordinate
   * (zero-based; x, y, z order) free_plane_val specifies the plane intercept
   * coordinate along that axis
   */
  unsigned int coordPlaneSliceGetDistance(
      unsigned int free_plane_index, double free_plane_val,
      EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXd> distances, unsigned int max_points) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<EsdfVoxel>::Ptr esdf_layer_;

  // Interpolator for the layer.
  Interpolator<EsdfVoxel> interpolator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_ESDF_MAP_H_
