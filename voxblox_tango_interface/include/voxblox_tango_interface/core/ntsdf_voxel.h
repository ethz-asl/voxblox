#ifndef VOXBLOX_TANGO_INTERFACE_CORE_VOXEL_H_
#define VOXBLOX_TANGO_INTERFACE_CORE_VOXEL_H_

#include <cstdint>
#include <string>

#include "voxblox/core/common.h"
#include "voxblox/core/color.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

/* NOTE(mereweth@jpl.nasa.gov) - This voxel type is not used for now.
 * The only implemented functionality is to load the Tango serialized NTSDF into
 * a Layer<TsdfVoxel>
 */
struct NtsdfVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint32_t ntsdf = 0;
  uint32_t color = 0;

  /* TODO(mereweth@jpl.nasa.gov) - should we use 16-bit fields
   * with attribute packed instead?
   */
};

// Used for serialization only.
namespace voxel_types {
  const std::string kNtsdf = "ntsdf";
}  // namespace voxel_types

template <>
inline std::string getVoxelType<NtsdfVoxel>() {
  return voxel_types::kNtsdf;
}

}  // namespace voxblox

#endif  // VOXBLOX_TANGO_INTERFACE_CORE_VOXEL_H_
