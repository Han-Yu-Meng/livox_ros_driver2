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

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  for (uint32_t i = 0; i < kMaxSourceLidar; ++i) {
    private_pub_[i] = nullptr;
    private_imu_pub_[i] = nullptr;
  }
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  cur_node_ = nullptr;
}

Lddc::~Lddc() {
  PrepareExit();
  std::cout << "lddc destroy" << std::endl;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }
  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, index);
    } else {
      // Unsupported transfer format in standalone library: drop packets
      static bool first_warn = true;
      if (first_warn) {
        std::cout << "warning: only livox custom message publishing is supported; dropping other formats" << std::endl;
        first_warn = false;
      }
      StoragePacket pkg;
      QueuePop(p_queue, &pkg);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
  }
}

// PointCloud2 publishing removed in standalone build.

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg.frame_id.assign(frame_id_);

  // standalone: no seq

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg.timebase = timestamp;

  livox_msg.stamp = timestamp;

  livox_msg.point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index);
    livox_msg.lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {
  if (custom_cb_) {
    custom_cb_(livox_msg);
  }
  Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));
  if (kOutputToRos == output_type_) {
    if (publisher_ptr) publisher_ptr->publish(livox_msg);
  }
}

// PCL message support removed in standalone build.

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
  imu_msg.header.frame_id = "livox_frame";

  timestamp = imu_data.time_stamp;
  imu_msg.header.stamp = timestamp;  // direct assign uint64_t

  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp);

  if (imu_cb_) {
    imu_cb_(imu_msg);
  }

  Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));
  if (kOutputToRos == output_type_) {
    if (publisher_ptr) publisher_ptr->publish(imu_msg);
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
    return global_imu_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {
  if (!cur_node_) {
    return PublisherPtr(nullptr);
  }
  if (kLivoxCustomMsg == msg_type) {
    DRIVER_INFO(*cur_node_, "%s publish use livox custom format", topic_name.c_str());
    return cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
  } else if (kLivoxImuMsg == msg_type) {
    DRIVER_INFO(*cur_node_, "%s publish use imu format", topic_name.c_str());
    return cur_node_->create_publisher<ImuMsg>(topic_name, queue_size);
  } else {
    PublisherPtr null_publisher(nullptr);
    return null_publisher;
  }
}

void Lddc::CreateBagFile(const std::string &file_name) {
  (void)file_name; // bag not supported in standalone build
}

}  // namespace livox_ros
