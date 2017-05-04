#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>

#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/integrator_utils_fast.h"

using namespace voxblox;  // NOLINT

class FastTsdfCastRayTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    LOG(INFO) << "Setting up the test environment.";


  }

  
};

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}