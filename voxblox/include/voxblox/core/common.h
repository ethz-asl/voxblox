#ifndef VOXBLOX_CORE_COMMON_H_
#define VOXBLOX_CORE_COMMON_H_

#include <deque>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>

namespace voxblox {

// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedQueue = std::queue<Type, AlignedDeque<Type>>;
template <typename Type>
using AlignedStack = std::stack<Type, AlignedDeque<Type>>;
template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                    std::forward<Arguments>(arguments)...);
}

// Types.
typedef float FloatingPoint;
typedef int IndexElement;
typedef int64_t LongIndexElement;

typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Ray;

typedef Eigen::Matrix<IndexElement, 3, 1> AnyIndex;
typedef AnyIndex VoxelIndex;
typedef AnyIndex BlockIndex;
typedef AnyIndex SignedIndex;

typedef Eigen::Matrix<LongIndexElement, 3, 1> LongIndex;
typedef LongIndex GlobalIndex;

typedef std::pair<BlockIndex, VoxelIndex> VoxelKey;

typedef AlignedVector<AnyIndex> IndexVector;
typedef IndexVector BlockIndexList;
typedef IndexVector VoxelIndexList;
typedef AlignedVector<LongIndex> LongIndexVector;
typedef LongIndexVector GlobalIndexVector;

// Common containers for indices (used in FIESTA)
typedef AlignedQueue<GlobalIndex> GlobalIndexQueue;
typedef AlignedList<GlobalIndex> GlobalIndexList;
typedef std::unordered_set<GlobalIndex> GlobalIndexSet;

struct Color;
struct Label;

// Pointcloud types for external interface.
typedef AlignedVector<Point> Pointcloud;
typedef AlignedVector<Color> Colors;
typedef AlignedVector<Label> Labels;

// For triangle meshing/vertex access.
typedef size_t VertexIndex;
typedef AlignedVector<VertexIndex> VertexIndexList;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Triangle;
typedef AlignedVector<Triangle> TriangleVector;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
    Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint> Rotation;
typedef kindr::minimal::RotationQuaternionTemplate<
    FloatingPoint>::Implementation Quaternion;

// For alignment of layers / point clouds
typedef Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> PointsMatrix;
template <size_t size>
using SquareMatrix = Eigen::Matrix<FloatingPoint, size, size>;

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


// Constants used across the library.
constexpr FloatingPoint kEpsilon = 1e-6; /**< Used for coordinates. */
constexpr float kFloatEpsilon = 1e-6;    /**< Used for weights. */
constexpr int kKITTIMaxIntstance = 1000;    /**< Used for assign an unqiue panoptic label. */

struct Label {
  Label() : sem_label(0), ins_label(0) {}
  Label(short int _sem_label, short int _ins_label) 
      : sem_label(_sem_label), ins_label(_ins_label) {}
  Label(uint32_t label) {
    full_label = label;
    sem_label = label & 0xFFFF; 
    ins_label = label >> 16; 
    // TODO(py): to do a better hashing or increase the number of 1000 here
    id_label = sem_label * kKITTIMaxIntstance + ins_label; 
    // name = semanticKittiLabelNameLUT(sem_label);
  }

  int id_label; 
  uint32_t full_label;
  short int sem_label; //int16_t
  short int ins_label; //int16_t
  //std::string name;
};


// Grid <-> point conversion functions.

/**
 * NOTE: Due the limited accuracy of the FloatingPoint type, this
 * function doesn't always compute the correct grid index for coordinates
 * near the grid cell boundaries. Use the safer `getGridIndexFromOriginPoint` if
 * the origin point is available.
 */
template <typename IndexType>
inline IndexType getGridIndexFromPoint(const Point& point,
                                       const FloatingPoint grid_size_inv) {
  return IndexType(std::floor(point.x() * grid_size_inv + kEpsilon),
                   std::floor(point.y() * grid_size_inv + kEpsilon),
                   std::floor(point.z() * grid_size_inv + kEpsilon));
}

/**
 * NOTE: Due the limited accuracy of the FloatingPoint type, this
 * function doesn't always compute the correct grid index for coordinates
 * near the grid cell boundaries.
 */
template <typename IndexType>
inline IndexType getGridIndexFromPoint(const Point& scaled_point) {
  return IndexType(std::floor(scaled_point.x() + kEpsilon),
                   std::floor(scaled_point.y() + kEpsilon),
                   std::floor(scaled_point.z() + kEpsilon));
}

/**
 * NOTE: This function is safer than `getGridIndexFromPoint`, because it assumes
 * we pass in not an arbitrary point in the grid cell, but the ORIGIN. This way
 * we can avoid the floating point precision issue that arrises for calls to
 * `getGridIndexFromPoint`for arbitrary points near the border of the grid cell.
 */
template <typename IndexType>
inline IndexType getGridIndexFromOriginPoint(
    const Point& point, const FloatingPoint grid_size_inv) {
  return IndexType(std::round(point.x() * grid_size_inv),
                   std::round(point.y() * grid_size_inv),
                   std::round(point.z() * grid_size_inv));
}

template <typename IndexType>
inline Point getCenterPointFromGridIndex(const IndexType& idx,
                                         FloatingPoint grid_size) {
  return Point((static_cast<FloatingPoint>(idx.x()) + 0.5) * grid_size,
               (static_cast<FloatingPoint>(idx.y()) + 0.5) * grid_size,
               (static_cast<FloatingPoint>(idx.z()) + 0.5) * grid_size);
}

template <typename IndexType>
inline Point getOriginPointFromGridIndex(const IndexType& idx,
                                         FloatingPoint grid_size) {
  return Point(static_cast<FloatingPoint>(idx.x()) * grid_size,
               static_cast<FloatingPoint>(idx.y()) * grid_size,
               static_cast<FloatingPoint>(idx.z()) * grid_size);
}

/**
 * Converts between Block + Voxel index and GlobalVoxelIndex.
 * Note that this takes int VOXELS_PER_SIDE, and
 * getBlockIndexFromGlobalVoxelIndex takes voxels per side inverse.
 */
inline GlobalIndex getGlobalVoxelIndexFromBlockAndVoxelIndex(
    const BlockIndex& block_index, const VoxelIndex& voxel_index,
    int voxels_per_side) {
  return GlobalIndex(block_index.cast<LongIndexElement>() * voxels_per_side +
                     voxel_index.cast<LongIndexElement>());
}

inline BlockIndex getBlockIndexFromGlobalVoxelIndex(
    const GlobalIndex& global_voxel_idx, FloatingPoint voxels_per_side_inv) {
  return BlockIndex(
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.x()) *
                 voxels_per_side_inv),
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.y()) *
                 voxels_per_side_inv),
      std::floor(static_cast<FloatingPoint>(global_voxel_idx.z()) *
                 voxels_per_side_inv));
}

inline bool isPowerOfTwo(int x) { return (x & (x - 1)) == 0; }

/**
 * Converts from a global voxel index to the index inside a block.
 * NOTE: assumes that voxels_per_side is a power of 2 and uses a bitwise and as
 * a computationally cheap substitute for the modulus operator
 */
inline VoxelIndex getLocalFromGlobalVoxelIndex(
    const GlobalIndex& global_voxel_idx, const int voxels_per_side) {
  // add a big number to the index to make it positive
  constexpr int offset = 1 << (8 * sizeof(IndexElement) - 1);

  CHECK(isPowerOfTwo(voxels_per_side));

  return VoxelIndex((global_voxel_idx.x() + offset) & (voxels_per_side - 1),
                    (global_voxel_idx.y() + offset) & (voxels_per_side - 1),
                    (global_voxel_idx.z() + offset) & (voxels_per_side - 1));
}

inline void getBlockAndVoxelIndexFromGlobalVoxelIndex(
    const GlobalIndex& global_voxel_idx, const int voxels_per_side,
    BlockIndex* block_index, VoxelIndex* voxel_index) {
  CHECK_NOTNULL(block_index);
  CHECK_NOTNULL(voxel_index);
  const FloatingPoint voxels_per_side_inv = 1.0 / voxels_per_side;
  *block_index =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv);
  *voxel_index =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side);
}

// Math functions.
inline int signum(FloatingPoint x) { return (x == 0) ? 0 : x < 0 ? -1 : 1; }

// For occupancy/octomap-style mapping.
inline float logOddsFromProbability(float probability) {
  CHECK(probability >= 0.0f && probability <= 1.0f);
  return log(probability / (1.0 - probability));
}

inline float probabilityFromLogOdds(float log_odds) {
  return 1.0 - (1.0 / (1.0 + exp(log_odds)));
}

inline void transformPointcloud(const Transformation& T_N_O,
                                const Pointcloud& ptcloud,
                                Pointcloud* ptcloud_out) {
  ptcloud_out->clear();
  ptcloud_out->resize(ptcloud.size());

  for (size_t i = 0; i < ptcloud.size(); ++i) {
    (*ptcloud_out)[i] = T_N_O * ptcloud[i];
  }
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H_
