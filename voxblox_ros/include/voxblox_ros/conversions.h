#ifndef VOXBLOX_ROS_CONVERSIONS_H_
#define VOXBLOX_ROS_CONVERSIONS_H_

#include <std_msgs/ColorRGBA.h>

#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>

namespace voxblox {

void colorVoxbloxToMsg(const Color& color, std_msgs::ColorRGBA* color_msg) {
  CHECK_NOTNULL(color_msg);
  color_msg->r = color.r / 255.0;
  color_msg->g = color.g / 255.0;
  color_msg->b = color.b / 255.0;
  color_msg->a = color.a / 255.0;
}

void colorMsgToVoxblox(const std_msgs::ColorRGBA& color_msg, Color* color) {
  color->r = static_cast<uint8_t>(color_msg.r * 255.0);
  color->g = static_cast<uint8_t>(color_msg.g * 255.0);
  color->b = static_cast<uint8_t>(color_msg.b * 255.0);
  color->a = static_cast<uint8_t>(color_msg.a * 255.0);
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_VISUALIZATION_H_
