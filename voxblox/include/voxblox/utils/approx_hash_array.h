#ifndef VOXBLOX_UTILS_APPROX_HASH_ARRAY_H_
#define VOXBLOX_UTILS_APPROX_HASH_ARRAY_H_

#include <atomic>
#include <limits>
#include <vector>

#include "voxblox/core/common.h"

/**
 * These classes allocate a fixed size array and index it with a hash that is
 * masked so that only its first N bits are non zero. This can be
 * thought of as a fast rough approximation of a hash table.
 * There are several advantages and some very significant disadvantages
 * Advantages-
 * - Simple and blazing fast lockless thread-safe approximate sets
 * - Can be used to provide more fine grain locking of blocks for threading then
 * simply locking the entire layer
 * Disadvantages-
 * - Highly inefficient use of memory (allocates 2^N elements)
 * - Cannot discern between two different elements with the same hash
 * - If the hash of two elements have the same first N elements of their hash,
 * only one can be stored.
 */

namespace voxblox {

/**
 * Basic container, give in an index and get the element that was stored there.
 * There are 2^unmasked_bits elements in the container, which element is
 * returned depends on your hashing function.
 * Uses at least 2^unmasked_bits * sizeof(StoreElement) bytes of ram
 */
template <size_t unmasked_bits, typename StoredElement, typename IndexType,
          typename IndexTypeHasher>
class ApproxHashArray {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StoredElement& get(const size_t& hash) {
    return pseudo_map_[hash & bit_mask_];
  }

  StoredElement& get(const IndexType& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return get(*hash);
  }

  StoredElement& get(const IndexType& index) {
    size_t hash = hasher_(index);
    return get(hash);
  }

 private:
  static constexpr size_t pseudo_map_size_ = (1 << unmasked_bits);
  static constexpr size_t bit_mask_ = (1 << unmasked_bits) - 1;

  std::array<StoredElement, pseudo_map_size_> pseudo_map_;
  IndexTypeHasher hasher_;
};

/**
 * Acts as a fast and thread safe set, with the serious limitation of both
 * false-negatives and false positives being possible.
 * A false positive occurs if two different elements have the same hash, and the
 * other element was already added to the set.
 * A false negative occurs if an element was removed to add another element with
 * the same masked hash. The chance of this happening is inversely proportional
 * to 2^unmasked_bits.
 * Uses at least (2^unmasked_bits + full_reset_threshold) * sizeof(StoreElement)
 * bytes of ram.
 * Note that the reset function is not thread safe.
 */
template <size_t unmasked_bits, size_t full_reset_threshold, typename IndexType,
          typename IndexTypeHasher>
class ApproxHashSet {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ApproxHashSet() : offset_(0), pseudo_set_(pseudo_set_size_) {
    for (std::atomic<size_t>& value : pseudo_set_) {
      value.store(0, std::memory_order_relaxed);
    }
    // The array used for storing values is initialized with zeros. However, the
    // zeroth bin can actually store the 0 hash. Because of this to prevent a
    // false positive on looking up a 0 hash this bin needs to initially store a
    // different value.
    pseudo_set_[offset_].store(std::numeric_limits<size_t>::max());
  }

  /**
   * Returns true if an element with the same hash is currently in the set,
   * false otherwise.
   * Note due to the masking of bits, many elements that were previously
   * inserted into the ApproxHashSet have been overwritten by other values.
   */
  inline bool isHashCurrentlyPresent(const size_t& hash) {
    const size_t array_index = (hash & bit_mask_) + offset_;

    return (pseudo_set_[array_index].load(std::memory_order_relaxed) == hash);
  }

  inline bool isHashCurrentlyPresent(const IndexType& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return isHashCurrentlyPresent(*hash);
  }

  inline bool isHashCurrentlyPresent(const IndexType& index) {
    size_t hash = hasher_(index);
    return isHashCurrentlyPresent(hash);
  }
  /**
   * Returns true if it replaced the element in the masked_hash's with the hash
   * of the given element.
   * Returns false if this hash was already there and no replacement was needed.
   * THIS IS THE MOST EXPENSIVE FUNCTION IN ALL OF VOXBLOX.
   * PROILE AND TEST AFTER EVEN THE MOST SUPERFICIAL CHANGE
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n
   * Also note that while the below layout of ifs and variables may not appear
   * the most efficient the compiler seems to do some black magic with it that
   * makes it come out ahead of other formulations.
   */
  inline bool replaceHash(const size_t& hash) {
    const size_t array_index = (hash & bit_mask_) + offset_;

    if (pseudo_set_[array_index].load(std::memory_order_relaxed) == hash) {
      return false;
    } else {
      pseudo_set_[array_index].store(hash, std::memory_order_relaxed);
      return true;
    }
  }

  inline bool replaceHash(const IndexType& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return replaceHash(*hash);
  }

  inline bool replaceHash(const IndexType& index) {
    const size_t hash = hasher_(index);
    return replaceHash(hash);
  }

  /**
   * If unmasked_bits is large, the array takes a lot of memory, this makes
   * clearing it slow.
   * However offsetting which bin hashes are placed into has the same effect.
   * Once we run out of room to offset by (determined by full_reset_threshold)
   * we clear the memory).
   * This function is not thread safe.
   */
  void resetApproxSet() {
    if (++offset_ >= full_reset_threshold) {
      for (std::atomic<size_t>& value : pseudo_set_) {
        value.store(0, std::memory_order_relaxed);
      }
      offset_ = 0;

      // The array used for storing values is initialized with zeros. However,
      // the zeroth bin can actually store the 0 hash. Because of this to
      // prevent a false positive on looking up a 0 hash this bin needs to
      // initially store a different value.
      pseudo_set_[offset_].store(std::numeric_limits<size_t>::max());
    }
  }

 private:
  static constexpr size_t pseudo_set_size_ =
      (1 << unmasked_bits) + full_reset_threshold;
  static constexpr size_t bit_mask_ = (1 << unmasked_bits) - 1;

  size_t offset_;
  std::vector<std::atomic<size_t>> pseudo_set_;

  IndexTypeHasher hasher_;
};
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_APPROX_HASH_ARRAY_H_
