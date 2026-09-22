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

#ifndef LIVOX_DRIVER_PUB_HANDLER_H_
#define LIVOX_DRIVER_PUB_HANDLER_H_

#include <atomic>
#include <cstring>
#include <condition_variable> // std::condition_variable
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>              // std::mutex
#include <thread>
#include <unordered_map>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/queue_monitor.h"

namespace livox_ros {

class LidarPubHandler {
 public:
  LidarPubHandler();
  ~ LidarPubHandler() {}

  void PointCloudProcess(RawPacket& pkt);
  void SetLidarsExtParam(LidarExtParameter param);
  void GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds);
  void SetFilterConfig(const FilterConfig& filter_config) { filter_config_ = filter_config; }

  uint64_t GetRecentTimeStamp();
  uint32_t GetLidarPointCloudsSize();
  uint64_t GetLidarBaseTime();

  /**
   * 一次加锁取回缓存的时间范围与点数。用于替代分别调用
   * GetLidarBaseTime()/GetLidarRecentTimeStamp()，避免读到正在被 swap 的容器。
   * 缓存为空时返回 base_time=0, recent_time=0, size=0。
   */
  void GetPendingTimeRange(uint64_t& base_time, uint64_t& recent_time, uint32_t& size);

  /**
   * 切片取出：把 offset_time <= cutoff_time 的点搬到输出并移出缓存，
   * 剩余点保留等待下一次发布。用于在积压过大时切成正常大小的帧，
   * 避免一次性吐出超大帧。返回本次取出的点数。
   */
  uint32_t ExtractPointsUpTo(uint64_t cutoff_time, std::vector<PointXyzlt>& points_clouds);

 private:
  void LivoxLidarPointCloudProcess(RawPacket & pkt);
  void ProcessCartesianHighPoint(RawPacket & pkt);
  void ProcessCartesianLowPoint(RawPacket & pkt);
  void ProcessSphericalPoint(RawPacket & pkt);
  bool IsPointValid(uint8_t tag) const;
  std::vector<PointXyzlt> points_clouds_;
  FilterConfig filter_config_;
  ExtParameterDetailed extrinsic_ = {
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 1},
      {0, 0, 1}
    }
  };
  std::mutex mutex_;
  std::atomic_bool is_set_extrinsic_params_;
};
  
class PubHandler {
 public:
  using PointCloudsCallback = std::function<void(PointFrame*, void *)>;
  using ImuDataCallback = std::function<void(ImuData*, void*)>;
  using TimePoint = std::chrono::high_resolution_clock::time_point;

  PubHandler() {}

  ~ PubHandler() { Uninit(); }

  void Uninit();
  void RequestExit();
  void Init();
  void SetPointCloudConfig(const double publish_freq);
  void SetFilterConfig(const FilterConfig& filter_config);
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data);
  void AddLidarsExtParam(LidarExtParameter& extrinsic_params);
  void ClearAllLidarsExtrinsicParams();
  void SetImuDataCallback(ImuDataCallback cb, void* client_data);

 private:
  //thread to process raw data
  void RawDataProcess();
  std::atomic<bool> is_quit_{false};
  std::shared_ptr<std::thread> point_process_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;

  //publish callback
  void CheckTimer(uint32_t id);
  void PublishPointCloud();
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data);
  
  static bool GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id);
  static uint64_t GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size);

  /** 单个雷达待发布点云缓存的监测句柄（仅 RawDataProcess 线程访问）。 */
  QueueMonitor::Gauge* GetPendingGauge(uint32_t id);

  PointCloudsCallback points_callback_;
  void* pub_client_data_ = nullptr;

  ImuDataCallback imu_callback_;
  void* imu_client_data_ = nullptr;

  PointFrame frame_;

  std::deque<RawPacket> raw_packet_queue_;
  QueueMonitor::Gauge* raw_queue_gauge_ = nullptr;              /**< 原始 UDP 包队列监测。 */
  std::unordered_map<uint32_t, QueueMonitor::Gauge*> pending_gauges_; /**< 每个雷达的待发布点云缓存监测。 */

  //pub config
  uint64_t publish_interval_ = 100000000; //100 ms
  uint64_t publish_interval_tolerance_ = 100000000; //100 ms
  uint64_t publish_interval_ms_ = 100; //100 ms
  /** 缓存跨度超过该值即强制按 publish_interval_ 切片发布，避免突发巨帧。 */
  uint64_t max_frame_span_ns_ = 200000000; // 2 * publish_interval_
  TimePoint last_pub_time_;

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_;
  std::map<uint32_t, std::vector<PointXyzlt>> points_;
  /** 每个雷达上一次发布所处的绝对时间槽 recent_time / publish_interval_（仅处理线程访问）。 */
  std::unordered_map<uint32_t, uint64_t> last_publish_slot_;
  std::map<uint32_t, LidarExtParameter> lidar_extrinsics_;
  FilterConfig filter_config_;
  static std::atomic<bool> is_timestamp_sync_;
  uint16_t lidar_listen_id_ = 0;
};

PubHandler &pub_handler();

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_PUB_HANDLER_H_