#ifndef VOXBLOX_CORE_BLOCK_H_
#define VOXBLOX_CORE_BLOCK_H_

#include <memory>

#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/voxel_array.h"

namespace voxblox {

class BaseBlock {
 public:
  typedef std::shared_ptr<BaseBlock> Ptr;
  typedef std::shared_ptr<const BaseBlock> ConstPtr;

  BaseBlock(size_t num_layers, const Point& origin, FloatingPoint block_size)
      : num_layers_(num_layers),
        origin_(origin),
        block_size_(block_size),
        has_data_(false) {}

  virtual ~BaseBlock() {}

  const Point& getOrigin() const { return origin_; }

 protected:
  size_t num_layers_;
  Point origin_;
  FloatingPoint block_size_;

  // Is set to true if any one of the voxels in this block received an update.
  bool has_data_;
};
}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_H_
