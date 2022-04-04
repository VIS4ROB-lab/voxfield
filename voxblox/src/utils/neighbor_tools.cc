#include "voxblox/utils/neighbor_tools.h"

#include "voxblox/core/common.h"

namespace voxblox {

// clang-format off
const NeighborhoodLookupTables::Distances NeighborhoodLookupTables::kDistances = [] { // NOLINT
  const float sqrt_2 = std::sqrt(2);
  const float sqrt_3 = std::sqrt(3);
  Distances distance_matrix;
  distance_matrix <<  1.f, 1.f, 1.f, 1.f, 1.f, 1.f,
                      sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                      sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                      sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                      sqrt_3, sqrt_3, sqrt_3, sqrt_3,
                      sqrt_3, sqrt_3, sqrt_3, sqrt_3;
  return distance_matrix;
}();


const NeighborhoodLookupTables::IndexOffsets NeighborhoodLookupTables::kOffsets = [] { // NOLINT
  IndexOffsets directions_matrix;
  directions_matrix << -1,  1,  0,  0,  0,  0, -1, -1,  1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1, -1, -1,  1,  1,  1,  1, // NOLINT
                        0,  0, -1,  1,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1,  0,  0,  0,  0, -1, -1,  1,  1, -1, -1,  1,  1, // NOLINT
                        0,  0,  0,  0, -1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1, -1,  1, -1,  1, -1,  1, -1,  1; // NOLINT
  return directions_matrix;
}();

const NeighborhoodLookupTables::LongIndexOffsets NeighborhoodLookupTables::kLongOffsets = [] { // NOLINT
  return NeighborhoodLookupTables::kOffsets.cast<LongIndexElement>(); // NOLINT
}();
// clang-format on

}  // namespace voxblox
