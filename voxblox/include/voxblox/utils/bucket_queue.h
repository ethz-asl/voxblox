#ifndef VOXBLOX_UTILS_BUCKET_QUEUE_H_
#define VOXBLOX_UTILS_BUCKET_QUEUE_H_

#include <deque>
#include <queue>
#include <vector>

#include <glog/logging.h>

#include "voxblox/core/common.h"

namespace voxblox {
/**
 * Bucketed priority queue, mostly following L. Yatziv et al in
 * O(N) Implementation of the Fast Marching Algorithm, though skipping the
 * circular aspect (don't care about a bit more memory used for this).
 */
template <typename T>
class BucketQueue {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BucketQueue() : last_bucket_index_(0) {}
  explicit BucketQueue(int num_buckets, double max_val)
      : num_buckets_(num_buckets),
        max_val_(max_val),
        last_bucket_index_(0),
        num_elements_(0) {
    buckets_.resize(num_buckets_);
  }

  /// WARNING: will CLEAR THE QUEUE!
  void setNumBuckets(int num_buckets, double max_val) {
    max_val_ = max_val;
    num_buckets_ = num_buckets;
    buckets_.clear();
    buckets_.resize(num_buckets_);
    num_elements_ = 0;
  }

  void push(const T& key, double value) {
    CHECK_NE(num_buckets_, 0);
    if (value > max_val_) {
      value = max_val_;
    }
    int bucket_index =
        std::floor(std::abs(value) / max_val_ * (num_buckets_ - 1));
    if (bucket_index >= num_buckets_) {
      bucket_index = num_buckets_ - 1;
    }
    if (bucket_index < last_bucket_index_) {
      last_bucket_index_ = bucket_index;
    }
    buckets_[bucket_index].push(key);
    num_elements_++;
  }

  void pop() {
    if (empty()) {
      return;
    }
    while (buckets_[last_bucket_index_].empty() &&
           last_bucket_index_ < num_buckets_) {
      last_bucket_index_++;
    }
    if (last_bucket_index_ < num_buckets_) {
      buckets_[last_bucket_index_].pop();
      num_elements_--;
    }
  }

  T front() {
    CHECK_NE(num_buckets_, 0);
    CHECK(!empty());
    while (buckets_[last_bucket_index_].empty() &&
           last_bucket_index_ < num_buckets_) {
      last_bucket_index_++;
    }
    return buckets_[last_bucket_index_].front();
  }

  bool empty() { return num_elements_ == 0; }

  size_t size() { return num_elements_; }

  void clear() {
    buckets_.clear();
    buckets_.resize(num_buckets_);
    last_bucket_index_ = 0;
    num_elements_ = 0;
  }

 private:
  int num_buckets_;
  double max_val_;
  voxblox::AlignedVector<voxblox::AlignedQueue<T>> buckets_;

  /// Speeds up retrivals.
  int last_bucket_index_;
  /// This is also to speed up empty checks.
  size_t num_elements_;
};
}  // namespace voxblox
#endif  // VOXBLOX_UTILS_BUCKET_QUEUE_H_
