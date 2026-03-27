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

#ifndef LIVOX_DRIVER2_API_H_
#define LIVOX_DRIVER2_API_H_

#include <memory>
#include <string>
#include <functional>
#include "livox_custom_msg.h"

// Redefine or include the POD Imu message
namespace livox_ros {

using CustomMsg = livox_ros_driver2::CustomMsg;

#ifndef SENSOR_MSGS_IMU_STRUCT_
#define SENSOR_MSGS_IMU_STRUCT_
struct ImuHeader {
  std::string frame_id;
  uint64_t stamp; // nano seconds
};
struct ImuVector3 {
  double x;
  double y;
  double z;
};
struct ImuMsg {
  ImuHeader header;
  ImuVector3 angular_velocity;
  ImuVector3 linear_acceleration;
};
#endif

using CustomMsgCallback = std::function<void(const CustomMsg& msg)>;
using ImuMsgCallback = std::function<void(const ImuMsg& msg)>;

class LivoxDriver {
 public:
  static std::unique_ptr<LivoxDriver> Create();
  virtual ~LivoxDriver() = default;

  // 1. 加载配置接口
  virtual bool LoadConfig(const std::string& config_path) = 0;

  // 2. 启动/暂停接口
  virtual bool Start() = 0;
  virtual void Stop() = 0;

  // 3. 注册回调函数接口
  virtual void RegisterCustomMsgCallback(CustomMsgCallback cb) = 0;
  virtual void RegisterImuMsgCallback(ImuMsgCallback cb) = 0;
};

} // namespace livox_ros

#endif // LIVOX_DRIVER2_API_H_
