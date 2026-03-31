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
#include <future>

#include "include/livox_ros_driver2.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

// Standalone library initialization: construct a DriverNode and setup Lddc
namespace livox_ros {

DriverNode::DriverNode()
  : config_path_(""), is_started_(false)
{
  std::cout << "Livox Driver2 standalone library" << std::endl;

  // default parameters
  int xfer_format = kLivoxCustomMsg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToCallback;
  std::string frame_id = "livox_frame";

  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
}

std::unique_ptr<LivoxDriver> LivoxDriver::Create() {
  return std::make_unique<DriverNode>();
}

bool DriverNode::LoadConfig(const std::string& config_path) {
  config_path_ = config_path;
  return true;
}

void DriverNode::SetParameters(int multi_topic, double publish_freq, const std::string& frame_id) {
  if (lddc_ptr_) {
    lddc_ptr_->SetMultiTopic(multi_topic);
    lddc_ptr_->SetPublishFrq(publish_freq);
    lddc_ptr_->SetFrameId(frame_id);
  }
}

bool DriverNode::Start() {
  if (is_started_) return true;
  if (config_path_.empty()) {
    std::cerr << "Config path is empty, cannot start." << std::endl;
    return false;
  }

  future_ = exit_signal_.get_future();

  LdsLidar *read_lidar = LdsLidar::GetInstance(lddc_ptr_->GetPublishFrq());
  read_lidar->SetLddc(static_cast<void*>(lddc_ptr_.get()));
  lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
  if (!(read_lidar->InitLdsLidar(config_path_))) {
    std::cerr << "Init lds lidar failed!" << std::endl;
    return false;
  }
  std::cout << "Init lds lidar successfully!" << std::endl;

  is_started_ = true;
  return true;
}

void DriverNode::Stop() {
  if (!is_started_) return;
  exit_signal_.set_value();
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


void DriverNode::PointCloudDataPollThread() {}

void DriverNode::ImuDataPollThread() {}





















