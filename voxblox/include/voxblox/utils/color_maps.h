#ifndef VOXBLOX_UTILS_COLOR_MAPS_H_
#define VOXBLOX_UTILS_COLOR_MAPS_H_

#include "voxblox/core/common.h"
#include "voxblox/core/color.h"

namespace voxblox {

class ColorMap {
 public:
  ColorMap() : min_value_(0.0), max_value_(1.0) {}
  virtual ~ColorMap() {}

  void setMinValue(float min_value) { min_value_ = min_value; }

  void setMaxValue(float max_value) { max_value_ = max_value; }

  virtual Color colorLookup(float value) = 0;

 protected:
  float min_value_;
  float max_value_;
};

class GrayscaleColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return grayColorMap(new_value);
  }
};

class InverseGrayscaleColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return grayColorMap(1.0 - new_value);
  }
};

class RainbowColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return rainbowColorMap(new_value);
  }
};

class InverseRainbowColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return rainbowColorMap(1.0 - new_value);
  }
};

class IronbowColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);

    std::vector<Colors> palette_colors;
    palette_colors.push_back(Color(0, 0, 0));
    palette_colors.push_back(Color(145, 20, 145));
    palette_colors.push_back(Color(255, 138, 0));
    palette_colors.push_back(Color(255, 230, 40));
    palette_colors.push_back(Color(255, 255, 255));
    // Add an extra to avoid overflow.
    palette_colors.push_back(Color(255, 255, 255));

    float increment = 1.0 / (palette_colors.size() - 2);

    size_t index = static_cast<size_t>(std::floor(value/increment));

    Color color = Color::blendTwoColors(
        palette_colors[index], increment * (index + 1) - value,
        palette_colors[index + 1], value - increment * (index));

    return color;
  }
};

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_COLOR_MAPS_H_
