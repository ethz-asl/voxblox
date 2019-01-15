#ifndef VOXBLOX_UTILS_COLOR_MAPS_H_
#define VOXBLOX_UTILS_COLOR_MAPS_H_

#include <algorithm>
#include <vector>

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"

namespace voxblox {

class ColorMap {
 public:
  ColorMap() : min_value_(0.0), max_value_(1.0) {}
  virtual ~ColorMap() {}

  void setMinValue(float min_value) { min_value_ = min_value; }

  void setMaxValue(float max_value) { max_value_ = max_value; }

  virtual Color colorLookup(float value) const = 0;

 protected:
  float min_value_;
  float max_value_;
};

class GrayscaleColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) const {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return grayColorMap(new_value);
  }
};

class InverseGrayscaleColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) const {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return grayColorMap(1.0 - new_value);
  }
};

class RainbowColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) const {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return rainbowColorMap(new_value);
  }
};

class InverseRainbowColorMap : public ColorMap {
 public:
  virtual Color colorLookup(float value) const {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);
    return rainbowColorMap(1.0 - new_value);
  }
};

class IronbowColorMap : public ColorMap {
 public:
  IronbowColorMap() : ColorMap() {
    palette_colors_.push_back(Color(0, 0, 0));
    palette_colors_.push_back(Color(145, 20, 145));
    palette_colors_.push_back(Color(255, 138, 0));
    palette_colors_.push_back(Color(255, 230, 40));
    palette_colors_.push_back(Color(255, 255, 255));
    // Add an extra to avoid overflow.
    palette_colors_.push_back(Color(255, 255, 255));

    increment_ = 1.0 / (palette_colors_.size() - 2);
  }

  virtual Color colorLookup(float value) const {
    float new_value = std::min(max_value_, std::max(min_value_, value));
    new_value = (new_value - min_value_) / (max_value_ - min_value_);

    size_t index = static_cast<size_t>(std::floor(new_value / increment_));

    return Color::blendTwoColors(
        palette_colors_[index], increment_ * (index + 1) - new_value,
        palette_colors_[index + 1], new_value - increment_ * (index));
  }

 protected:
  std::vector<Color> palette_colors_;
  float increment_;
};

}  // namespace voxblox

#endif  // VOXBLOX_UTILS_COLOR_MAPS_H_
