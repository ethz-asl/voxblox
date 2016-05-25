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

// For triangle meshing/vertex access.
typedef size_t VertexIndex;
typedef std::vector<VertexIndex> VertexIndexList;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Triangle;
typedef std::vector<Triangle, Eigen::aligned_allocator<Triangle> >
    TriangleVector;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;

struct Color {
  Color() : r(0), g(0), b(0), a(0) {}
  Color(uint8_t _r, uint8_t _g, uint8_t _b)
      : Color(_r, _g, _b, 255) {}
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
                                       second_color.r * second_weight + 0.5);
    new_color.g = static_cast<uint8_t>(first_color.g * first_weight +
                                       second_color.g * second_weight + 0.5);
    new_color.b = static_cast<uint8_t>(first_color.b * first_weight +
                                       second_color.b * second_weight + 0.5);
    new_color.a = static_cast<uint8_t>(first_color.a * first_weight +
                                       second_color.a * second_weight + 0.5);

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

inline int signum(FloatingPoint x) { return (x == 0) ? 0 : x < 0 ? -1 : 1; }

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COMMON_H_
