#ifndef VOXBLOX_CORE_COLOR_H_
#define VOXBLOX_CORE_COLOR_H_

#include "voxblox/core/common.h"

namespace voxblox {

// Color maps.

/**
 * Maps an input h from a value between 0.0 and 1.0 into a rainbow. Copied from
 * OctomapProvider in octomap.
 */
inline Color rainbowColorMap(double h) {
  Color color;
  color.a = 255;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = 255 * v;
      color.g = 255 * n;
      color.b = 255 * m;
      break;
    case 1:
      color.r = 255 * n;
      color.g = 255 * v;
      color.b = 255 * m;
      break;
    case 2:
      color.r = 255 * m;
      color.g = 255 * v;
      color.b = 255 * n;
      break;
    case 3:
      color.r = 255 * m;
      color.g = 255 * n;
      color.b = 255 * v;
      break;
    case 4:
      color.r = 255 * n;
      color.g = 255 * m;
      color.b = 255 * v;
      break;
    case 5:
      color.r = 255 * v;
      color.g = 255 * m;
      color.b = 255 * n;
      break;
    default:
      color.r = 255;
      color.g = 127;
      color.b = 127;
      break;
  }

  return color;
}

/// Maps an input h from a value between 0.0 and 1.0 into a grayscale color.
inline Color grayColorMap(double h) {
  Color color;
  color.a = 255;

  color.r = round(h * 255);
  color.b = color.r;
  color.g = color.r;

  return color;
}

inline Color randomColor() {
  Color color;

  color.a = 255;

  color.r = rand_r() % 256;
  color.b = rand_r() % 256;
  color.g = rand_r() % 256;

  return color;
}

}  // namespace voxblox

#endif  // VOXBLOX_CORE_COLOR_H_
