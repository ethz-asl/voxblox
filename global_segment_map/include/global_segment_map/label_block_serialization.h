#ifndef GLOBAL_SEGMENT_MAP_LABEL_BLOCK_SERIALIZATION_H_
#define GLOBAL_SEGMENT_MAP_LABEL_BLOCK_SERIALIZATION_H_

#include <vector>

#include <voxblox/core/block.h>
#include <voxblox/core/common.h>

#include "global_segment_map/label_voxel.h"

namespace voxblox {

template <>
void Block<LabelVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data);

template <>
void Block<LabelVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const;

}  // namespace voxblox

#endif  // GLOBAL_SEGMENT_MAP_LABEL_BLOCK_SERIALIZATION_H_
