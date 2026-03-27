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

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"

#include "include/livox_driver2_api.h"
#include "lds.h"

namespace livox_ros {

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToCallback = 0,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;

/** Type-Definitions based on ROS versions */
// Publisher and message type mappings for standalone library
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using ImuMsg = livox_ros::ImuMsg; // Use API version

class Lddc final {
 public:
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void RegisterCustomMsgCallback(CustomMsgCallback cb) { custom_cb_ = cb; }
  void RegisterImuMsgCallback(ImuMsgCallback cb) { imu_cb_ = cb; }

  void DistributePointCloudData(void);
  void DistributeImuData(void);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }

  double GetPublishFrq() { return publish_frq_; }
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

 public:
  Lds *lds_;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
    void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

    // Only CustomMsg and Imu publishing are supported in standalone library
    void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index);
    void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

    void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index);
    void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg);

    void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp);

    void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num,
      uint32_t offset_time, uint32_t point_interval, uint32_t echo_num);

  void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index);

 private:
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;

  CustomMsgCallback custom_cb_;
  ImuMsgCallback imu_cb_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_H_
