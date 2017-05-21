#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

class FastUpdateTsdfTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // TODO(mfehr): IMPLEMENT
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FastUpdateTsdfTest, CompareWithBaseline) {
  // TODO(mfehr): IMPLEMENT
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
