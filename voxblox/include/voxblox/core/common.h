#ifndef VOXBLOX_CORE_COMMON_H_
#define VOXBLOX_CORE_COMMON_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>

namespace voxblox {

typedef double FloatingPoint;

typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Ray;

typedef Eigen::Vector3i AnyIndex;
typedef AnyIndex VoxelIndex;
typedef AnyIndex BlockIndex;

typedef std::vector<AnyIndex> IndexVector;
typedef IndexVector BlockIndexList;
typedef IndexVector VoxelIndexList;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;

struct Color {
  Color() : r(0), g(0), b(0), a(0) {}
  Color(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a)
      : r(_r), g(_g), b(_b), a(_a) {}

  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;

  static Color blendTwoColors(const Color& first_color,
                              FloatingPoint first_weight,
                              const Color& second_color,
                              FloatingPoint second_weight) {
    FloatingPoint total_weight = first_weight + second_weight;

    first_weight /= total_weight;
    second_weight /= total_weight;

    Color new_color;
    new_color.r = static_cast<uint8_t>(first_color.r * first_weight +
                                       second_color.r * second_weight);
    new_color.g = static_cast<uint8_t>(first_color.g * first_weight +
                                       second_color.g * second_weight);
    new_color.b = static_cast<uint8_t>(first_color.b * first_weight +
                                       second_color.b * second_weight);
    new_color.a = static_cast<uint8_t>(first_color.a * first_weight +
                                       second_color.a * second_weight);

    return new_color;
  }
};

// Pointcloud types for external interface.
typedef std::vector<Point> Pointcloud;
typedef std::vector<Color> Colors;

// TODO(mfehr, helenol): Potentially slow, fix this.
inline Eigen::Vector3i floorVectorAndDowncast(
    const Eigen::Matrix<FloatingPoint, 3, 1>& vector) {
  return Eigen::Vector3i(static_cast<int>(std::floor(vector.x())),
                         static_cast<int>(std::floor(vector.y())),
                         static_cast<int>(std::floor(vector.z())));
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H_
