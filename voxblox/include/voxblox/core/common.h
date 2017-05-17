#ifndef VOXBLOX_CORE_COMMON_H_
#define VOXBLOX_CORE_COMMON_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>

namespace voxblox {

// Types.
typedef double FloatingPoint;
typedef int IndexElement;

typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Ray;

typedef Eigen::Matrix<IndexElement, 3, 1> AnyIndex;
typedef AnyIndex VoxelIndex;
typedef AnyIndex BlockIndex;

typedef std::pair<BlockIndex, VoxelIndex> VoxelKey;

typedef std::vector<AnyIndex, Eigen::aligned_allocator<AnyIndex>> IndexVector;
typedef IndexVector BlockIndexList;
typedef IndexVector VoxelIndexList;

struct Color;
typedef uint32_t Label;

// Pointcloud types for external interface.
typedef std::vector<Point, Eigen::aligned_allocator<Point>> Pointcloud;
typedef std::vector<Color> Colors;
typedef std::vector<Label> Labels;

// For triangle meshing/vertex access.
typedef size_t VertexIndex;
typedef std::vector<VertexIndex> VertexIndexList;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Triangle;
typedef std::vector<Triangle, Eigen::aligned_allocator<Triangle>>
    TriangleVector;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
    Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint> Rotation;

// For alignment of layers / point clouds
typedef Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> PointsMatrix;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Matrix3;

// Interpolation structure
typedef Eigen::Matrix<FloatingPoint, 8, 8> InterpTable;
typedef Eigen::Matrix<FloatingPoint, 1, 8> InterpVector;
// Type must allow negatives:
typedef Eigen::Array<IndexElement, 3, 8> InterpIndexes;

struct Color {
  Color() : r(0), g(0), b(0), a(0) {}
  Color(uint8_t _r, uint8_t _g, uint8_t _b) : Color(_r, _g, _b, 255) {}
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
    new_color.r = static_cast<uint8_t>(
        round(first_color.r * first_weight + second_color.r * second_weight));
    new_color.g = static_cast<uint8_t>(
        round(first_color.g * first_weight + second_color.g * second_weight));
    new_color.b = static_cast<uint8_t>(
        round(first_color.b * first_weight + second_color.b * second_weight));
    new_color.a = static_cast<uint8_t>(
        round(first_color.a * first_weight + second_color.a * second_weight));

    return new_color;
  }

  // Now a bunch of static colors to use! :)
  static const Color White() { return Color(255, 255, 255); }
  static const Color Black() { return Color(0, 0, 0); }
  static const Color Gray() { return Color(127, 127, 127); }
  static const Color Red() { return Color(255, 0, 0); }
  static const Color Green() { return Color(0, 255, 0); }
  static const Color Blue() { return Color(0, 0, 255); }
  static const Color Yellow() { return Color(255, 255, 0); }
  static const Color Orange() { return Color(255, 127, 0); }
  static const Color Purple() { return Color(127, 0, 255); }
  static const Color Teal() { return Color(0, 255, 255); }
  static const Color Pink() { return Color(255, 0, 127); }
};

// Grid <-> point conversion functions.

// IMPORTANT NOTE: Due the limited accuracy of the FloatingPoint type, this
// function doesn't always compute the correct grid index for coordinates
// near the grid cell boundaries.
inline AnyIndex getGridIndexFromPoint(const Point& point,
                                      const FloatingPoint grid_size_inv) {
  const FloatingPoint kEpsilon = 1e-6;
  return AnyIndex(std::floor(point.x() * grid_size_inv + kEpsilon),
                 std::floor(point.y() * grid_size_inv + kEpsilon),
                 std::floor(point.z() * grid_size_inv + kEpsilon));
}

// IMPORTANT NOTE: Due the limited accuracy of the FloatingPoint type, this
// function doesn't always compute the correct grid index for coordinates
// near the grid cell boundaries.
inline AnyIndex getGridIndexFromPoint(const Point& scaled_point) {
  return AnyIndex(std::floor(scaled_point.x()), std::floor(scaled_point.y()),
                  std::floor(scaled_point.z()));
}

inline AnyIndex getGridIndexFromOriginPoint(const Point& point,
                                            const FloatingPoint grid_size_inv) {
  return AnyIndex(std::round(point.x() * grid_size_inv),
                  std::round(point.y() * grid_size_inv),
                  std::round(point.z() * grid_size_inv));
}

inline Point getCenterPointFromGridIndex(const AnyIndex& idx,
                                         FloatingPoint grid_size) {
  return Point((static_cast<FloatingPoint>(idx.x()) + 0.5) * grid_size,
               (static_cast<FloatingPoint>(idx.y()) + 0.5) * grid_size,
               (static_cast<FloatingPoint>(idx.z()) + 0.5) * grid_size);
}

inline Point getOriginPointFromGridIndex(const AnyIndex& idx,
                                         FloatingPoint grid_size) {
  return Point(static_cast<FloatingPoint>(idx.x()) * grid_size,
               static_cast<FloatingPoint>(idx.y()) * grid_size,
               static_cast<FloatingPoint>(idx.z()) * grid_size);
}

inline BlockIndex getBlockIndexFromGlobalVoxelIndex(
    const AnyIndex& global_voxel_idx, FloatingPoint voxels_per_side_inv_) {
  return BlockIndex(
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.x()) *
                 voxels_per_side_inv_),
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.y()) *
                 voxels_per_side_inv_),
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.z()) *
                 voxels_per_side_inv_));
}

inline VoxelIndex getLocalFromGlobalVoxelIndex(const AnyIndex& global_voxel_idx,
                                               int voxels_per_side) {
  VoxelIndex local_voxel_idx(global_voxel_idx.x() % voxels_per_side,
                             global_voxel_idx.y() % voxels_per_side,
                             global_voxel_idx.z() % voxels_per_side);

  // Make sure we're within bounds.
  for (unsigned int i = 0u; i < 3u; ++i) {
    if (local_voxel_idx(i) < 0) {
      local_voxel_idx(i) += voxels_per_side;
    }
  }

  return local_voxel_idx;
}

// Math functions.
inline int signum(FloatingPoint x) { return (x == 0) ? 0 : x < 0 ? -1 : 1; }

// For occupancy/octomap-style mapping.
inline float logOddsFromProbability(float probability) {
  DCHECK(probability >= 0.0f && probability <= 1.0f);
  return log(probability / (1.0 - probability));
}

inline float probabilityFromLogOdds(float log_odds) {
  return 1.0 - (1.0 / (1.0 + exp(log_odds)));
}

template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                    std::forward<Arguments>(arguments)...);
}
}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H_
