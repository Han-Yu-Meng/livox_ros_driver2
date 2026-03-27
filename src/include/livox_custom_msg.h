// Simple POD definitions to replace ROS custom messages
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace livox_ros_driver2 {

struct CustomPoint {
  uint32_t offset_time;
  float x;
  float y;
  float z;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;
};

struct CustomMsg {
  // simplified header replacements
  std::string frame_id;
  uint64_t stamp = 0;

  uint64_t timebase = 0;
  uint32_t point_num = 0;
  uint8_t lidar_id = 0;
  uint8_t rsvd[3] = {0,0,0};
  std::vector<CustomPoint> points;
};

} // namespace livox_ros_driver2
