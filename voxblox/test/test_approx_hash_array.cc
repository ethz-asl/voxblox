#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/utils/approx_hash_array.h"

using namespace voxblox;  // NOLINT

class ApproxHashArrayTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
};

TEST_F(ApproxHashArrayTest, InsertValues) {
  // storing 256 ints
  ApproxHashArray<8, int, AnyIndex, AnyIndexHash> approx_hash_array;

  // write 512 values, first 256 should be overwritten
  for (int i = 0; i < 512; ++i) {
    approx_hash_array.get(i) = i;
  }

  for (int i = 0; i < 256; ++i) {
    EXPECT_EQ(approx_hash_array.get(i), i + 256);
  }
}

TEST_F(ApproxHashArrayTest, ArrayRandomWriteRead) {
  // storing 65536 size_ts
  ApproxHashArray<16, size_t, AnyIndex, AnyIndexHash> approx_hash_array;

  // generate 1000 random elements
  AlignedVector<AnyIndex> rand_indexes;
  std::mt19937 gen(1);
  std::uniform_int_distribution<> dis(1, std::numeric_limits<int>::max());
  for (int i = 0; i < 1000; ++i) {
    rand_indexes.push_back(AnyIndex(dis(gen), dis(gen), dis(gen)));
  }

  // insert their hash
  AnyIndexHash hasher;
  for (const AnyIndex& rand_index : rand_indexes) {
    size_t hash = hasher(rand_index);
    approx_hash_array.get(rand_index) = hash;
  }

  // find out how many we can recover
  int recovered = 0;
  for (const AnyIndex& rand_index : rand_indexes) {
    size_t hash = hasher(rand_index);
    if (approx_hash_array.get(rand_index) == hash) {
      ++recovered;
    }
  }

  // require at least a 95% success rate
  EXPECT_GT(recovered, 950);
}

TEST_F(ApproxHashArrayTest, SetRandomWriteRead) {
  // storing 65536 size_ts
  ApproxHashSet<16, 10, AnyIndex, AnyIndexHash> approx_hash_set;

  // generate 1000 random elements
  AlignedVector<AnyIndex> rand_indexes;
  std::mt19937 gen(1);
  std::uniform_int_distribution<> dis(1, std::numeric_limits<int>::max());
  for (int i = 0; i < 1000; ++i) {
    rand_indexes.push_back(AnyIndex(dis(gen), dis(gen), dis(gen)));
  }

  // test insert and clearing
  for (size_t i = 0; i < 50u; ++i) {
    approx_hash_set.resetApproxSet();

    // insert their values
    int inserted = 0;
    for (const AnyIndex& rand_index : rand_indexes) {
      if (approx_hash_set.replaceHash(rand_index)) {
        ++inserted;
      }
    }
    // require at least a 95% success rate
    EXPECT_GT(inserted, 950);

    // insert a second time
    inserted = 0;
    for (const AnyIndex& rand_index : rand_indexes) {
      if (approx_hash_set.replaceHash(rand_index)) {
        ++inserted;
      }
    }
    // require at least a 95% failure rate
    EXPECT_LT(inserted, 50);
  }

  // find out how many we can recover
  int recovered = 0;
  for (const AnyIndex& rand_index : rand_indexes) {
    AnyIndexHash hasher;
    size_t hash = hasher(rand_index);
    if (approx_hash_set.isHashCurrentlyPresent(hash)) {
      ++recovered;
    }
  }

  // require at least a 95% success rate
  EXPECT_GT(recovered, 950);
}

TEST_F(ApproxHashArrayTest, SetRandomWriteReadLongIndex) {
  // storing 65536 size_ts
  ApproxHashSet<16, 10, LongIndex, LongIndexHash> approx_hash_set;

  // generate 1000 random elements
  AlignedVector<LongIndex> rand_indexes;
  std::mt19937 gen(1);
  std::uniform_int_distribution<int64_t> dis(
      1, std::numeric_limits<int64_t>::max());
  for (int i = 0; i < 1000; ++i) {
    rand_indexes.push_back(LongIndex(dis(gen), dis(gen), dis(gen)));
  }

  // test insert and clearing
  for (size_t i = 0; i < 50u; ++i) {
    approx_hash_set.resetApproxSet();

    // insert their values
    unsigned int inserted = 0;
    for (const LongIndex& rand_index : rand_indexes) {
      if (approx_hash_set.replaceHash(rand_index)) {
        ++inserted;
      }
    }
    // require at least a 95% success rate
    EXPECT_GT(inserted, 950u);

    // insert a second time
    inserted = 0;
    for (const LongIndex& rand_index : rand_indexes) {
      if (approx_hash_set.replaceHash(rand_index)) {
        ++inserted;
      }
    }
    // require at least a 95% failure rate
    EXPECT_LT(inserted, 50u);
  }

  // find out how many we can recover
  unsigned int recovered = 0;
  for (const LongIndex& rand_index : rand_indexes) {
    LongIndexHash hasher;
    size_t hash = hasher(rand_index);
    if (approx_hash_set.isHashCurrentlyPresent(hash)) {
      ++recovered;
    }
  }

  // require at least a 95% success rate
  EXPECT_GT(recovered, 950u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
