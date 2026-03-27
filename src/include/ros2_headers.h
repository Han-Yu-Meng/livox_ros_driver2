//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// Lightweight non-ROS stubs and logging for standalone build

#ifndef ROS2_HEADERS_H_
#define ROS2_HEADERS_H_

#include <thread>
#include <future>
#include <memory>
#include <functional>
#include <iostream>
#include <string>

#include "livox_driver2_api.h"

// Simple logging macros (ignore node parameter)
#define DRIVER_DEBUG(node, ...) do { std::cout << "DEBUG: "; printf(__VA_ARGS__); std::cout << std::endl; } while(0)
#define DRIVER_INFO(node, ...) do { std::cout << "INFO: "; printf(__VA_ARGS__); std::cout << std::endl; } while(0)
#define DRIVER_WARN(node, ...) do { std::cout << "WARN: "; printf(__VA_ARGS__); std::cout << std::endl; } while(0)
#define DRIVER_ERROR(node, ...) do { std::cerr << "ERROR: "; printf(__VA_ARGS__); std::cerr << std::endl; } while(0)
#define DRIVER_FATAL(node, ...) do { std::cerr << "FATAL: "; printf(__VA_ARGS__); std::cerr << std::endl; } while(0)

// sensor_msgs and pcl types are not available in standalone mode; provide minimal placeholders
namespace sensor_msgs {
namespace msg {

struct Header {
std::string frame_id;
// timestamp (uses rclcpp::Time stub)
uint64_t stamp; // nano seconds
};

struct Imu {
Header header;
struct Vector3 { double x; double y; double z; } angular_velocity, linear_acceleration;
};

} // namespace msg
} // namespace sensor_msgs

using ImuMsg = livox_ros::ImuMsg; // consistent name for lddc.cpp

#endif // ROS2_HEADERS_H_
