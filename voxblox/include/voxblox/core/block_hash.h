#ifndef VOXBLOX_CORE_BLOCK_HASH_H_
#define VOXBLOX_CORE_BLOCK_HASH_H_

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

/**
 * Performs deco hashing on block indexes. Based on recommendations of
 * "Investigating the impact of Suboptimal Hashing Functions" by L. Buckley et
 * al.
 */
struct AnyIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// number was arbitrarily chosen with no good justification
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const AnyIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

template <typename ValueType>
struct AnyIndexHashMapType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      AnyIndex, ValueType, AnyIndexHash, std::equal_to<AnyIndex>,
      Eigen::aligned_allocator<std::pair<const AnyIndex, ValueType> > >
      type;
};

typedef std::unordered_set<AnyIndex, AnyIndexHash, std::equal_to<AnyIndex>,
                           Eigen::aligned_allocator<AnyIndex> >
    IndexSet;

typedef typename AnyIndexHashMapType<IndexVector>::type HierarchicalIndexMap;

typedef typename AnyIndexHashMapType<IndexSet>::type HierarchicalIndexSet;

typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;

/// Hash for large index values, see AnyIndexHash.
struct LongIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const LongIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

template <typename ValueType>
struct LongIndexHashMapType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      LongIndex, ValueType, LongIndexHash, std::equal_to<LongIndex>,
      Eigen::aligned_allocator<std::pair<const LongIndex, ValueType> > >
      type;
};

typedef std::unordered_set<LongIndex, LongIndexHash, std::equal_to<LongIndex>,
                           Eigen::aligned_allocator<LongIndex> >
    LongIndexSet;

}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_HASH_H_
