#ifndef VOXBLOX_ROS_THRESHOLD_H_
#define VOXBLOX_ROS_THRESHOLD_H_

namespace voxblox {
template <typename T>
class Threshold {
 public:
  // Constructors
  Threshold() : is_set_(false) {}
  explicit Threshold(const T threshold)
      : is_set_(true), threshold_(threshold) {}

  // Set the threshold
  void set(const T new_threshold) {
    is_set_ = true;
    threshold_ = new_threshold;
  }
  Threshold& operator=(const T& new_threshold) {
    set(new_threshold);
    return *this;
  }

  // Query if the threshold has been set
  bool isSet() const { return is_set_; }

  // Unset a threshold that has previously been set
  void unset() { is_set_ = false; }

  // Test if the threshold is less than, greater than, or equal to a value
  bool isSetAndLT(const T value) const { return is_set_ && threshold_ < value; }
  bool isSetAndLE(const T value) const {
    return is_set_ && threshold_ <= value;
  }
  bool isSetAndGT(const T value) const { return is_set_ && value < threshold_; }
  bool isSetAndGE(const T value) const {
    return is_set_ && value <= threshold_;
  }
  bool isSetAndEQ(const T value) const {
    return is_set_ && threshold_ == value;
  }

 private:
  bool is_set_;
  T threshold_;
};
}  // namespace voxblox

#endif  // VOXBLOX_ROS_THRESHOLD_H_
