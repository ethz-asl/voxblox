#ifndef VOXBLOX_FAST_BLOCK_HASH_H_
#define VOXBLOX_FAST_BLOCK_HASH_H_

#include <functional>
#include <unordered_map>

#include <Eigen/Core>

#include "voxblox_fast/core/common.h"

namespace voxblox_fast {

struct BlockIndexHash {
  static constexpr size_t prime1 = 73856093;
  static constexpr size_t prime2 = 19349663;
  static constexpr size_t prime3 = 83492791;

  std::size_t operator()(const BlockIndex& index) const {
    return (static_cast<unsigned int>(index.x()) * prime1 ^ index.y() * prime2 ^
            index.z() * prime3);
  }
};

template <typename ValueType>
struct BlockHashMapType {
  typedef std::unordered_map<
      BlockIndex, ValueType, BlockIndexHash, std::equal_to<BlockIndex>,
      Eigen::aligned_allocator<std::pair<const BlockIndex, ValueType> > >
      type;
};

typedef std::unordered_set<AnyIndex, BlockIndexHash, std::equal_to<AnyIndex>,
                           Eigen::aligned_allocator<AnyIndex> >
    IndexSet;

typedef typename BlockHashMapType<IndexVector>::type HierarchicalIndexMap;

typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;

}  // namespace voxblox

#endif  // VOXBLOX_FAST_BLOCK_HASH_H_
