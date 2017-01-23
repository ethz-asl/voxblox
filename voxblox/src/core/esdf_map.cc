#include "voxblox/core/esdf_map.h"

namespace voxblox {

bool EsdfMap::getDistanceAtPosition(const Eigen::Vector3d& position,
                                    double* distance) const {}

bool EsdfMap::getDistanceAndGradientAtPosition(
    const Eigen::Vector3d& position, double* distance,
    Eigen::Vector3d* gradient) const {}

}  // namespace voxblox
