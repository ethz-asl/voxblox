#ifndef VOXBLOX_CORE_ESDF_MAP_H_
#define VOXBLOX_CORE_ESDF_MAP_H_

#include <glog/logging.h>
#include <memory>
#include <string>
#include <utility>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/interpolator/interpolator.h"

#include "voxblox/io/layer_io.h"

namespace voxblox {

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

  explicit EsdfMap(const std::string& file_path)
      : esdf_layer_(
            io::LoadOrCreateLayerHeader<EsdfVoxel>(file_path, 0.2, 16u)),
        interpolator_(esdf_layer_.get()) {
    if (!io::LoadBlocksFromFile<EsdfVoxel>(
            file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kProhibit,
            esdf_layer_.get())) {
      // TODO(mereweth@jpl.nasa.gov) - throw std exception for Python to catch?
      throw std::runtime_error(std::string("Invalid file path: ") + file_path);
    }
    block_size_ = esdf_layer_->block_size();
  }

  // Creates a new EsdfMap based on a COPY of this layer.
  explicit EsdfMap(const Layer<EsdfVoxel>& layer)
      : EsdfMap(aligned_shared<Layer<EsdfVoxel>>(layer)) {}

  // Creates a new EsdfMap that contains this layer.
  explicit EsdfMap(Layer<EsdfVoxel>::Ptr layer)
      : esdf_layer_(layer), interpolator_(CHECK_NOTNULL(esdf_layer_.get())) {
    block_size_ = layer->block_size();
  }

  virtual ~EsdfMap() {}

  Layer<EsdfVoxel>* getEsdfLayerPtr() { return esdf_layer_.get(); }
  const Layer<EsdfVoxel>& getEsdfLayer() const { return *esdf_layer_; }

  FloatingPoint block_size() const { return block_size_; }
  FloatingPoint voxel_size() const { return esdf_layer_->voxel_size(); }

  // Specific accessor functions for esdf maps.
  // Returns true if the point exists in the map AND is observed.
  // These accessors use Vector3d and doubles explicitly rather than
  // FloatingPoint to have a standard, cast-free interface to planning
  // functions.
  bool getDistanceAtPosition(const Eigen::Vector3d& position,
                             double* distance) const;

  bool getDistanceAndGradientAtPosition(const Eigen::Vector3d& position,
                                        double* distance,
                                        Eigen::Vector3d* gradient) const;

  bool isObserved(const Eigen::Vector3d& position) const;

  // Convenience functions for querying many points at once from Python

  // TODO(mereweth@jpl.nasa.gov) - double check that position can not be mutated
  // EigenDRef is fully dynamic stride type alias for Numpy array slices
  // Use column-major matrices; column-by-column traversal is faster

  // convenience alias borrowed from pybind11
  using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  template <typename MatrixType>
  using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

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

  unsigned int coordPlaneSliceGetDistance(
      unsigned int free_plane_index, double free_plane_val,
      EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
      Eigen::Ref<Eigen::VectorXd> distances) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<EsdfVoxel>::Ptr esdf_layer_;

  // Interpolator for the layer.
  Interpolator<EsdfVoxel> interpolator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_CORE_ESDF_MAP_H_
