#ifndef UTILS_APPROX_HASH_ARRAY_H_
#define UTILS_APPROX_HASH_ARRAY_H_

#include <atomic>
#include <limits>
#include "voxblox/core/common.h"

// These classes allocate a fixed size array and index it with a hash that is
// masked so that only its first N bits are non zero. This can be
// thought of as a fast rough approximation of a hash table.
// There are several advantages and some very significant disadvantages
// Advantages-
// - Simple and blazing fast lockless thread-safe approximate sets
// - Can be used to provide more fine grain locking of blocks for threading then
// simply locking the entire layer
// Disadvantages-
// - Highly inefficient use of memory (allocates 2^N elements)
// - Cannot disern between two different elements with the same hash
// - If the hash of two elements have the same first N elements of their hash,
// only one can be store.

// Basic container, given in an index and get the element that was stored there.
// There are 2^unmasked_bits_ elements in the container, which element is
// returned depends on your hashing function.
// Uses at least 2^unmaksed_bits * sizeof(StoreElement) bytes of ram

namespace voxblox {
template <size_t unmasked_bits_, typename StoredElement>
class ApproxHashArray {
 public:
  StoredElement& get(const size_t& hash) {
    return pseudo_map_[hash & bit_mask_];
  }

  StoredElement& get(const AnyIndex& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return get(hash);
  }

  StoredElement& get(const AnyIndex& index) {
    size_t hash = hasher_(index);
    return get(hash);
  }

 private:
  static constexpr size_t pseudo_map_size_ = (1 << unmasked_bits_);
  static constexpr size_t bit_mask_ = (1 << unmasked_bits_) - 1;

  std::array<StoredElement, pseudo_map_size_> pseudo_map_;
  BlockIndexHash hasher_;
};

// Acts as a fast and thread safe set, with the serious limitation of both
// false-negatives and false positives being possible.
// Uses at least (2^unmaksed_bits + full_reset_threshold) * sizeof(StoreElement)
// bytes of ram.
// Note that the reset function is not thread safe.
template <size_t unmasked_bits_, size_t full_reset_threshold_>
class ApproxHashSet {
 public:
  ApproxHashSet() {
    pseudo_set_ptr_ = &pseudo_set_[reset_counter_++];

    // we init our set with zeros, except for the 0 bin which needs a different
    // number
    pseudo_set_ptr_[0].store(std::numeric_limits<size_t>::max());
  }

  // Returns true if an element with the same hash is currently in the set,
  // false otherwise
  // Note due to the masking of bits, many elements that were previously
  // inserted into the ApproxHashSet have been overwritten by other values.
  bool isHashCurrentlyPresent(const size_t& hash) {
    if (pseudo_set_ptr_[hash & bit_mask_].load(std::memory_order_relaxed) ==
        hash) {
      return true;
    } else {
      return false;
    }
  }

  bool isHashCurrentlyPresent(const AnyIndex& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return isHashCurrentlyPresent(hash);
  }

  bool isHashCurrentlyPresent(const AnyIndex& index) {
    size_t hash = hasher_(index);
    return isHashCurrentlyPresent(hash);
  }

  // Returns true if it replaced the element in the masked_hash's with the hash
  // of the given element
  // Returns false if this hash was already there and no replacement was needed
  bool replaceHash(const size_t& hash) {
    const size_t masked_hash = hash & bit_mask_;
    if (pseudo_set_ptr_[masked_hash].load(std::memory_order_relaxed) == hash) {
      return false;
    } else {
      pseudo_set_ptr_[masked_hash].store(hash, std::memory_order_relaxed);
      return true;
    }
  }

  bool replaceHash(const AnyIndex& index, size_t* hash) {
    DCHECK(hash);
    *hash = hasher_(index);
    return replaceHash(hash);
  }

  bool replaceHash(const AnyIndex& index) {
    size_t hash = hasher_(index);
    return replaceHash(hash);
  }

  // If unmasked_bits_ is large, the array takes a lot of memory, this makes
  // clearing it slow.
  // However offsetting which bin hashes are placed into have the same effect.
  // Once we run out of room to offset by (determined by full_reset_threshold we
  // clear the memory).
  // This function is not thread safe.
  void resetApproxSet() {
    if (reset_counter_ > full_reset_threshold_) {
      for (std::atomic<size_t>& value : pseudo_set_) {
        value.store(0, std::memory_order_relaxed);
      }
      reset_counter_ = 0;
      pseudo_set_ptr_ = &pseudo_set_[reset_counter_++];

      // we init our set with zeros, except for the 0 bin which needs a
      // different number
      pseudo_set_ptr_[0].store(std::numeric_limits<size_t>::max());

    } else {
      pseudo_set_ptr_ = &pseudo_set_[reset_counter_++];
    }
  }

 private:
  static constexpr size_t pseudo_set_size_ =
      (1 << unmasked_bits_) + full_reset_threshold_;
  static constexpr size_t bit_mask_ = (1 << unmasked_bits_) - 1;

  size_t reset_counter_;
  std::array<std::atomic<size_t>, pseudo_set_size_> pseudo_set_;
  std::atomic<size_t>* pseudo_set_ptr_;
  BlockIndexHash hasher_;
};
}  // namespace voxblox

#endif  // UTILS_APPROX_HASH_ARRAY_H_
