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

#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include "include/ros_headers.h"
#include "include/livox_driver2_api.h"

namespace livox_ros {

class Lddc;

// Standalone DriverNode (non-ROS) used by the C++ library
class DriverNode final : public LivoxDriver {
 public:
  DriverNode();
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  // API implementations
  bool LoadConfig(const std::string& config_path) override;
  bool Start() override;
  void Stop() override;
  void RegisterCustomMsgCallback(CustomMsgCallback cb) override;
  void RegisterImuMsgCallback(ImuMsgCallback cb) override;

  DriverNode& GetNode() noexcept;

  void PointCloudDataPollThread();
  void ImuDataPollThread();

  template <typename MessageT>
  std::shared_ptr<rclcpp::Publisher<MessageT>> create_publisher(const std::string &name, size_t queue_size) {
    return node_.create_publisher<MessageT>(name, queue_size);
  }

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

 private:
  rclcpp::Node node_;  // lightweight node stub for publisher creation and logging
  std::string config_path_;
  bool is_started_;
};

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H