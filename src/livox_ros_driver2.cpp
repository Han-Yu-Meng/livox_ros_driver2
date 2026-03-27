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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

// Standalone library initialization: construct a DriverNode and setup Lddc
namespace livox_ros {

DriverNode::DriverNode()
  : node_(), config_path_(""), is_started_(false)
{
  DRIVER_INFO(node_, "Livox Driver2 standalone library");

  // default parameters
  int xfer_format = kLivoxCustomMsg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame";

  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);
}

std::unique_ptr<LivoxDriver> LivoxDriver::Create() {
  return std::make_unique<DriverNode>();
}

bool DriverNode::LoadConfig(const std::string& config_path) {
  config_path_ = config_path;
  return true;
}

bool DriverNode::Start() {
  if (is_started_) return true;
  if (config_path_.empty()) {
    DRIVER_ERROR(node_, "Config path is empty, cannot start.");
    return false;
  }

  future_ = exit_signal_.get_future();

  LdsLidar *read_lidar = LdsLidar::GetInstance(lddc_ptr_->GetPublishFrq());
  lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
  if (!(read_lidar->InitLdsLidar(config_path_))) {
    DRIVER_ERROR(node_, "Init lds lidar failed!");
    return false;
  }
  DRIVER_INFO(node_, "Init lds lidar successfully!");

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  is_started_ = true;
  return true;
}

void DriverNode::Stop() {
  if (!is_started_) return;
  exit_signal_.set_value();
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  lddc_ptr_->PrepareExit();
  is_started_ = false;
}

void DriverNode::RegisterCustomMsgCallback(CustomMsgCallback cb) {
  lddc_ptr_->RegisterCustomMsgCallback(cb);
}

void DriverNode::RegisterImuMsgCallback(ImuMsgCallback cb) {
  lddc_ptr_->RegisterImuMsgCallback(cb);
}

} // namespace livox_ros


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}





















