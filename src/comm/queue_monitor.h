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

// 轻量级流水线监测：
//   1) 各缓冲队列（无界 deque / 有界环形队列 / 点云累积缓存）的深度、峰值、
//      入队、出队、丢弃、滞留时间；
//   2) 点云帧的输出心跳（用于发现"长时间不发布，然后一次性爆发"）；
//   3) CheckTimer 跳过发布的原因计数（用于定位发布门控被卡在哪一步）；
//   4) 下游回调耗时（用于发现消费端阻塞整个流水线）。
//
// 该模块只在观测接入点上做原子计数和一次加锁级的采样，不改变任何发布语义。
// 所有 Gauge* 为 nullptr 时接口自动降级为空操作，可以放心插桩。

#ifndef LIVOX_ROS_DRIVER_QUEUE_MONITOR_H_
#define LIVOX_ROS_DRIVER_QUEUE_MONITOR_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace livox_ros {

/** 一个被监测队列的实时数据句柄，生命周期跟随 QueueMonitor 单例。 */
struct Gauge {
  Gauge(const std::string& gauge_name, uint64_t cap, uint64_t warn)
      : name(gauge_name), capacity(cap), warn_level(warn) {}

  const std::string name;
  const uint64_t capacity;   /**< 队列容量，0 表示无界队列。 */
  const uint64_t warn_level; /**< 手动告警水位，0 表示按 capacity 的 3/4 推断。 */

  std::atomic<uint64_t> depth{0};
  std::atomic<uint64_t> peak{0};
  std::atomic<uint64_t> enqueue{0};
  std::atomic<uint64_t> dequeue{0};
  std::atomic<uint64_t> drop{0};
  std::atomic<uint64_t> last_change_ms{0}; /**< 最后一次深度变化的时间戳(ms)。 */
};

/** 队列快照，供外部读取。 */
struct GaugeSnapshot {
  std::string name;
  uint64_t capacity = 0;
  uint64_t depth = 0;
  uint64_t peak = 0;
  uint64_t enqueue = 0;
  uint64_t dequeue = 0;
  uint64_t drop = 0;
  uint64_t stale_ms = 0;
  bool valid = false;
};

/** 点云输出侧的统计快照。 */
struct PublishSnapshot {
  uint64_t frames = 0;
  uint64_t points_total = 0;
  uint64_t points_avg = 0;
  uint64_t points_max = 0;
  double interval_max_ms = 0.0;
  double callback_last_ms = 0.0;
  double callback_max_ms = 0.0;
  uint64_t stall_ms = 0;
};

class QueueMonitor {
 public:
  using Clock = std::chrono::steady_clock;

  /** CheckTimer 未发布的原因分类。 */
  enum class SkipReason : uint32_t {
    kSlotNotAdvanced = 0,    /**< 时间同步模式：时间戳尚未跨入下一个发布槽。 */
    kSpanTooShort,           /**< 时间同步模式：累积时长不足 publish_interval_tolerance。 */
    kEmptyCloud,             /**< 取出的点云为空。 */
    kIntervalNotReached,     /**< 非同步模式：wall clock 未到发布时间。 */
    kBootstrap,              /**< 首帧初始化。 */
    kForceFlush,             /**< 积压超过阈值，触发切片发布保护（非跳过，是限流出帧）。 */
    kReasonCount
  };
  static constexpr uint32_t kSkipReasonCount =
      static_cast<uint32_t>(SkipReason::kReasonCount);

  struct Config {
    bool enable = true;
    uint32_t period_ms = 1000;         /**< watchdog 采样/打印周期。 */
    uint32_t stall_warn_ms = 300;      /**< 超过该时长没有点云输出即告警。 */
    uint32_t callback_warn_ms = 50;    /**< 单次下游回调耗时告警阈值。 */
    uint64_t pending_points_warn = 250000; /**< 单个雷达缓存点数告警阈值。 */
    bool periodic_log = true;          /**< 周期打印一行摘要。 */
  };

  static QueueMonitor& Instance();
  ~QueueMonitor();

  /** 必须在 Start() 之前调用；运行期间修改不保证生效。 */
  void Configure(const Config& config);
  void Start();
  void Stop();
  bool IsRunning() const { return running_.load(std::memory_order_acquire); }

  /** 注册（或复用）一个队列采集点，返回稳定的句柄指针。 */
  Gauge* RegisterQueue(const std::string& name, uint64_t capacity = 0,
                       uint64_t warn_level = 0);

  /** 以下计数接口均线程安全，Gauge* 为 nullptr 时为空操作。 */
  void ReportEnqueue(Gauge* gauge, uint64_t n = 1);
  void ReportDequeue(Gauge* gauge, uint64_t n = 1);
  void ReportDrop(Gauge* gauge, uint64_t n = 1);
  void SetDepth(Gauge* gauge, uint64_t depth);
  void ResetGauge(Gauge* gauge);

  /** 一次点云帧发布成功。 */
  void NotifyFrame(uint64_t points);
  /** 下游回调（或任何被测耗时环节）的一次耗时，单位 ms。 */
  void NotifyCallbackCost(double cost_ms);
  /** CheckTimer 跳过发布的原因计数。 */
  void BumpSkip(SkipReason reason) {
    const uint32_t idx = static_cast<uint32_t>(reason);
    if (idx < kSkipReasonCount) {
      skip_[idx].fetch_add(1, std::memory_order_relaxed);
    }
  }
  uint64_t GetSkipCount(SkipReason reason) const {
    const uint32_t idx = static_cast<uint32_t>(reason);
    return (idx < kSkipReasonCount) ? skip_[idx].load(std::memory_order_relaxed) : 0;
  }

  /** 多行完整快照，可直接打印到日志。 */
  std::string Dump() const;
  /** 单行摘要，watchdog 周期打印使用。 */
  std::string DumpOneLine() const;
  PublishSnapshot GetPublishSnapshot() const;
  GaugeSnapshot GetGaugeSnapshot(const std::string& name) const;
  uint64_t GetPendingPointsWarn() const { return cfg_.pending_points_warn; }
  const Config& GetConfig() const { return cfg_; }
  void ResetPeaks();

  static uint64_t NowMs();

 private:
  QueueMonitor();
  QueueMonitor(const QueueMonitor&) = delete;
  QueueMonitor& operator=(const QueueMonitor&) = delete;

  void WatchdogLoop();
  void Tick();
  void CheckQueueWarn(Gauge* gauge, uint64_t now_ms);
  void CheckPublishStall(uint64_t now_ms, const std::vector<Gauge*>& gauges);
  void LogThrottled(const std::string& key, const std::string& message,
                    uint64_t min_interval_ms);
  Gauge* FindQueueLocked(const std::string& name) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::unique_ptr<Gauge>> gauges_;
  std::vector<Gauge*> gauge_list_;

  Config cfg_;
  const Clock::time_point base_tp_;
  std::atomic<bool> started_{false};
  std::atomic<bool> running_{false};
  std::thread watchdog_thread_;
  std::mutex cv_mutex_;
  std::condition_variable cv_;

  std::atomic<uint64_t> frames_{0};
  std::atomic<uint64_t> points_total_{0};
  std::atomic<uint64_t> points_max_{0};
  std::atomic<uint64_t> last_frame_ms_{0};
  std::atomic<double> interval_max_ms_{0.0};
  std::atomic<double> callback_last_ms_{0.0};
  std::atomic<double> callback_max_ms_{0.0};
  std::atomic<uint64_t> skip_[kSkipReasonCount];

  /** 仅 watchdog 线程访问，无需加锁。 */
  std::unordered_map<std::string, uint64_t> last_warn_ms_;
  std::unordered_map<std::string, uint64_t> last_enqueue_seen_;
  bool stall_reported_ = false;
  uint64_t stall_peak_ms_ = 0;
  double cb_max_warned_ = 0.0;
  uint64_t last_log_ms_ = 0;
};

/** RAII 计时器：析构时把耗时上报给 QueueMonitor。 */
class CallbackCostTimer {
 public:
  CallbackCostTimer() : begin_(QueueMonitor::Clock::now()) {}
  ~CallbackCostTimer() {
    const double cost_ms =
        std::chrono::duration<double, std::milli>(
            QueueMonitor::Clock::now() - begin_).count();
    QueueMonitor::Instance().NotifyCallbackCost(cost_ms);
  }

 private:
  QueueMonitor::Clock::time_point begin_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_QUEUE_MONITOR_H_
