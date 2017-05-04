#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>
#include <gtest/gtest.h>

#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator_fast.h"

using namespace voxblox;  // NOLINT

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}