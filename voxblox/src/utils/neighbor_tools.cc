#include "voxblox/utils/neighbor_tools.h"

#include "voxblox/core/common.h"

namespace voxblox {

// clang-format off
const NeighborhoodLookupTables::Distances NeighborhoodLookupTables::kDistances = [] {
  Distances distance_matrix;
  distance_matrix <<  1.f, 1.f, 1.f, 1.f, 1.f, 1.f,

                      std::sqrt(2), std::sqrt(2), std::sqrt(2), std::sqrt(2),
                      std::sqrt(2), std::sqrt(2), std::sqrt(2), std::sqrt(2),
                      std::sqrt(2), std::sqrt(2), std::sqrt(2), std::sqrt(2),

                      std::sqrt(3), std::sqrt(3), std::sqrt(3), std::sqrt(3),
                      std::sqrt(3), std::sqrt(3), std::sqrt(3), std::sqrt(3);
  return distance_matrix;
}();


const NeighborhoodLookupTables::IndexOffsets NeighborhoodLookupTables::kOffsets = [] {
  IndexOffsets directions_matrix;
  directions_matrix << -1,  1,  0,  0,  0,  0, -1, -1,  1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1, -1, -1,  1,  1,  1,  1,
                        0,  0, -1,  1,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1,  0,  0,  0,  0, -1, -1,  1,  1, -1, -1,  1,  1,
                        0,  0,  0,  0, -1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1, -1,  1, -1,  1, -1,  1, -1,  1;
  return directions_matrix;
}();

const NeighborhoodLookupTables::LongIndexOffsets NeighborhoodLookupTables::kLongOffsets = [] {
  return NeighborhoodLookupTables::kOffsets.cast<LongIndexElement>();
}();
// clang-format on

}  // namespace voxblox
