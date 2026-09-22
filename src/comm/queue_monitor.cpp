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

#include "comm/queue_monitor.h"

#include <stdio.h>

#include <iomanip>
#include <sstream>

namespace livox_ros {

namespace {

constexpr const char* kSkipReasonName[QueueMonitor::kSkipReasonCount] = {
    "slot_not_advanced", "span_too_short", "empty_cloud", "interval_not_reached",
    "bootstrap", "force_flush"};

void UpdateMaxU64(std::atomic<uint64_t>& target, uint64_t value) {
  uint64_t cur = target.load(std::memory_order_relaxed);
  while (value > cur &&
         !target.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
    // spin
  }
}

void UpdateMaxDouble(std::atomic<double>& target, double value) {
  double cur = target.load(std::memory_order_relaxed);
  while (value > cur &&
         !target.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
    // spin
  }
}

}  // namespace

QueueMonitor& QueueMonitor::Instance() {
  static QueueMonitor instance;
  return instance;
}

QueueMonitor::QueueMonitor() : base_tp_(Clock::now()) {
  for (uint32_t i = 0; i < kSkipReasonCount; ++i) {
    skip_[i].store(0, std::memory_order_relaxed);
  }
}

QueueMonitor::~QueueMonitor() { Stop(); }

uint64_t QueueMonitor::NowMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now() - Instance().base_tp_).count());
}

void QueueMonitor::Configure(const Config& config) { cfg_ = config; }

void QueueMonitor::Start() {
  if (!cfg_.enable) {
    return;
  }
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true)) {
    return;
  }
  running_.store(true, std::memory_order_release);
  watchdog_thread_ = std::thread(&QueueMonitor::WatchdogLoop, this);
  fprintf(stderr, "[LivoxMon] started, period=%ums, stall_warn=%ums, "
                  "callback_warn=%ums, pending_warn=%llu points\n",
          cfg_.period_ms, cfg_.stall_warn_ms, cfg_.callback_warn_ms,
          static_cast<unsigned long long>(cfg_.pending_points_warn));
}

void QueueMonitor::Stop() {
  {
    std::lock_guard<std::mutex> lock(cv_mutex_);
    running_.store(false, std::memory_order_release);
  }
  cv_.notify_all();
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  started_.store(false, std::memory_order_release);
}

Gauge* QueueMonitor::RegisterQueue(const std::string& name, uint64_t capacity,
                                   uint64_t warn_level) {
  std::lock_guard<std::mutex> lock(mutex_);
  Gauge* existed = FindQueueLocked(name);
  if (existed != nullptr) {
    return existed;
  }
  auto gauge = std::make_unique<Gauge>(name, capacity, warn_level);
  Gauge* raw = gauge.get();
  gauges_[name] = std::move(gauge);
  gauge_list_.push_back(raw);
  return raw;
}

Gauge* QueueMonitor::FindQueueLocked(const std::string& name) const {
  auto it = gauges_.find(name);
  return (it == gauges_.end()) ? nullptr : it->second.get();
}

void QueueMonitor::ReportEnqueue(Gauge* gauge, uint64_t n) {
  if (gauge == nullptr) {
    return;
  }
  UpdateMaxU64(gauge->peak, gauge->depth.load(std::memory_order_relaxed) + n);
  gauge->depth.fetch_add(n, std::memory_order_relaxed);
  gauge->enqueue.fetch_add(n, std::memory_order_relaxed);
  gauge->last_change_ms.store(NowMs(), std::memory_order_relaxed);
}

void QueueMonitor::ReportDequeue(Gauge* gauge, uint64_t n) {
  if (gauge == nullptr) {
    return;
  }
  const uint64_t depth = gauge->depth.load(std::memory_order_relaxed);
  gauge->depth.store(depth > n ? depth - n : 0, std::memory_order_relaxed);
  gauge->dequeue.fetch_add(n, std::memory_order_relaxed);
  gauge->last_change_ms.store(NowMs(), std::memory_order_relaxed);
}

void QueueMonitor::ReportDrop(Gauge* gauge, uint64_t n) {
  if (gauge == nullptr) {
    return;
  }
  gauge->drop.fetch_add(n, std::memory_order_relaxed);
  gauge->last_change_ms.store(NowMs(), std::memory_order_relaxed);
}

void QueueMonitor::SetDepth(Gauge* gauge, uint64_t depth) {
  if (gauge == nullptr) {
    return;
  }
  const uint64_t old = gauge->depth.exchange(depth, std::memory_order_relaxed);
  UpdateMaxU64(gauge->peak, depth);
  if (old != depth) {
    gauge->last_change_ms.store(NowMs(), std::memory_order_relaxed);
  }
}

void QueueMonitor::ResetGauge(Gauge* gauge) {
  if (gauge == nullptr) {
    return;
  }
  gauge->depth.store(0, std::memory_order_relaxed);
  gauge->enqueue.store(0, std::memory_order_relaxed);
  gauge->dequeue.store(0, std::memory_order_relaxed);
  gauge->drop.store(0, std::memory_order_relaxed);
  gauge->peak.store(0, std::memory_order_relaxed);
}

void QueueMonitor::NotifyFrame(uint64_t points) {
  const uint64_t now_ms = NowMs();
  const uint64_t prev_ms = last_frame_ms_.exchange(now_ms, std::memory_order_relaxed);
  frames_.fetch_add(1, std::memory_order_relaxed);
  points_total_.fetch_add(points, std::memory_order_relaxed);
  UpdateMaxU64(points_max_, points);
  if (prev_ms != 0 && now_ms > prev_ms) {
    UpdateMaxDouble(interval_max_ms_, static_cast<double>(now_ms - prev_ms));
  }
}

void QueueMonitor::NotifyCallbackCost(double cost_ms) {
  callback_last_ms_.store(cost_ms, std::memory_order_relaxed);
  UpdateMaxDouble(callback_max_ms_, cost_ms);
}

PublishSnapshot QueueMonitor::GetPublishSnapshot() const {
  PublishSnapshot snap;
  snap.frames = frames_.load(std::memory_order_relaxed);
  snap.points_total = points_total_.load(std::memory_order_relaxed);
  snap.points_max = points_max_.load(std::memory_order_relaxed);
  snap.points_avg = (snap.frames > 0) ? (snap.points_total / snap.frames) : 0;
  snap.interval_max_ms = interval_max_ms_.load(std::memory_order_relaxed);
  snap.callback_last_ms = callback_last_ms_.load(std::memory_order_relaxed);
  snap.callback_max_ms = callback_max_ms_.load(std::memory_order_relaxed);
  const uint64_t last = last_frame_ms_.load(std::memory_order_relaxed);
  const uint64_t now_ms = NowMs();
  snap.stall_ms = (last != 0 && now_ms > last) ? (now_ms - last) : 0;
  return snap;
}

GaugeSnapshot QueueMonitor::GetGaugeSnapshot(const std::string& name) const {
  GaugeSnapshot snap;
  std::lock_guard<std::mutex> lock(mutex_);
  Gauge* gauge = FindQueueLocked(name);
  if (gauge == nullptr) {
    return snap;
  }
  snap.valid = true;
  snap.name = gauge->name;
  snap.capacity = gauge->capacity;
  snap.depth = gauge->depth.load(std::memory_order_relaxed);
  snap.peak = gauge->peak.load(std::memory_order_relaxed);
  snap.enqueue = gauge->enqueue.load(std::memory_order_relaxed);
  snap.dequeue = gauge->dequeue.load(std::memory_order_relaxed);
  snap.drop = gauge->drop.load(std::memory_order_relaxed);
  const uint64_t last = gauge->last_change_ms.load(std::memory_order_relaxed);
  const uint64_t now_ms = NowMs();
  snap.stale_ms = (last != 0 && now_ms > last) ? (now_ms - last) : 0;
  return snap;
}

void QueueMonitor::ResetPeaks() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (Gauge* gauge : gauge_list_) {
    if (gauge != nullptr) {
      gauge->peak.store(gauge->depth.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
    }
  }
  callback_max_ms_.store(0.0, std::memory_order_relaxed);
  interval_max_ms_.store(0.0, std::memory_order_relaxed);
  points_max_.store(0, std::memory_order_relaxed);
}

void QueueMonitor::WatchdogLoop() {
  while (running_.load(std::memory_order_acquire)) {
    {
      std::unique_lock<std::mutex> lock(cv_mutex_);
      cv_.wait_for(lock, std::chrono::milliseconds(cfg_.period_ms),
                   [this] { return !running_.load(std::memory_order_acquire); });
    }
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }
    Tick();
  }
}

void QueueMonitor::Tick() {
  const uint64_t now_ms = NowMs();

  std::vector<Gauge*> snapshot_list;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_list = gauge_list_;
  }

  for (Gauge* gauge : snapshot_list) {
    CheckQueueWarn(gauge, now_ms);
  }
  CheckPublishStall(now_ms, snapshot_list);

  if (cfg_.callback_warn_ms > 0) {
    const double cb_max = callback_max_ms_.load(std::memory_order_relaxed);
    if (cb_max > static_cast<double>(cfg_.callback_warn_ms) && cb_max > cb_max_warned_) {
      cb_max_warned_ = cb_max;
      std::ostringstream oss;
      oss << "[LivoxMon][WARN] consumer callback is slow: peak=" << std::fixed
          << std::setprecision(1) << cb_max << " ms (threshold "
          << cfg_.callback_warn_ms
          << " ms). 回调阻塞会冻结整条流水线并导致点云突发。";
      LogThrottled("callback_slow", oss.str(), 5000);
    }
  }

  if (cfg_.periodic_log &&
      (last_log_ms_ == 0 || now_ms - last_log_ms_ >= cfg_.period_ms)) {
    last_log_ms_ = now_ms;
    fprintf(stderr, "%s\n", DumpOneLine().c_str());
  }
}

void QueueMonitor::CheckQueueWarn(Gauge* gauge, uint64_t now_ms) {
  if (gauge == nullptr) {
    return;
  }
  const uint64_t depth = gauge->depth.load(std::memory_order_relaxed);
  const uint64_t cap = gauge->capacity;
  uint64_t warn_level = gauge->warn_level;
  if (warn_level == 0 && cap > 0) {
    warn_level = cap * 3 / 4;
  }

  std::ostringstream oss;
  if (cap > 0 && depth >= cap) {
    oss << "[LivoxMon][WARN] queue '" << gauge->name << "' FULL: depth=" << depth
        << "/" << cap << ", dropped=" << gauge->drop.load(std::memory_order_relaxed)
        << ". 缓存写满，新帧被静默丢弃。";
    LogThrottled(gauge->name + ":full", oss.str(), 2000);
  } else if (warn_level > 0 && depth >= warn_level) {
    oss << "[LivoxMon][WARN] queue '" << gauge->name << "' accumulating: depth="
        << depth << (cap > 0 ? ("/" + std::to_string(cap)) : std::string("(unbounded)"))
        << " >= warn " << warn_level
        << ", enq=" << gauge->enqueue.load(std::memory_order_relaxed)
        << " deq=" << gauge->dequeue.load(std::memory_order_relaxed)
        << ". 生产快于消费，数据正在堆积。";
    LogThrottled(gauge->name + ":high", oss.str(), 2000);
  }

  const uint64_t drop = gauge->drop.load(std::memory_order_relaxed);
  const std::string drop_key = gauge->name + ":drop";
  uint64_t& drop_seen = last_enqueue_seen_[drop_key];
  if (drop > drop_seen) {
    if (drop_seen != 0) {
      std::ostringstream dss;
      dss << "[LivoxMon][WARN] queue '" << gauge->name << "' dropped "
          << (drop - drop_seen) << " frames since last check, total=" << drop
          << ". 检查消费线程是否被阻塞。";
      LogThrottled(drop_key, dss.str(), 5000);
    }
    drop_seen = drop;
  }

  // 队列长时间不变且非空：卡死征兆
  if (depth > 0) {
    const uint64_t last = gauge->last_change_ms.load(std::memory_order_relaxed);
    if (last != 0 && now_ms > last && (now_ms - last) > 3000) {
      std::ostringstream sss;
      sss << "[LivoxMon][WARN] queue '" << gauge->name << "' stalled: depth="
          << depth << " unchanged for " << (now_ms - last) << " ms. 流水线消费端可能卡住。";
      LogThrottled(gauge->name + ":stale", sss.str(), 5000);
    }
  }
}

void QueueMonitor::CheckPublishStall(uint64_t now_ms,
                                     const std::vector<Gauge*>& gauges) {
  const uint64_t frames = frames_.load(std::memory_order_relaxed);
  if (frames == 0) {
    return;
  }
  const uint64_t last = last_frame_ms_.load(std::memory_order_relaxed);
  if (last == 0) {
    return;
  }
  const uint64_t stall = (now_ms > last) ? (now_ms - last) : 0;

  if (stall > cfg_.stall_warn_ms) {
    if (stall > stall_peak_ms_) {
      stall_peak_ms_ = stall;
    }
    // 判断雷达侧是否仍在收数据：看原始包队列的入队计数是否还在增长。
    Gauge* raw = nullptr;
    for (Gauge* gauge : gauges) {
      if (gauge != nullptr && gauge->name == "raw_packet") {
        raw = gauge;
        break;
      }
    }
    std::ostringstream oss;
    oss << "[LivoxMon][WARN] no point cloud published for " << stall << " ms"
        << " (threshold " << cfg_.stall_warn_ms << " ms), frames=" << frames
        << ", max_interval=" << std::fixed << std::setprecision(1)
        << interval_max_ms_.load(std::memory_order_relaxed) << " ms"
        << ", callback_last=" << callback_last_ms_.load(std::memory_order_relaxed)
        << " ms";
    for (Gauge* gauge : gauges) {
      if (gauge != nullptr && gauge->name.compare(0, 11, "pending_pts") == 0) {
        oss << ", " << gauge->name << "="
            << gauge->depth.load(std::memory_order_relaxed);
      }
    }
    if (raw != nullptr) {
      const uint64_t enq = raw->enqueue.load(std::memory_order_relaxed);
      uint64_t& enq_seen = last_enqueue_seen_["raw_packet:enq"];
      const uint64_t delta = (enq > enq_seen && enq_seen != 0) ? (enq - enq_seen) : enq;
      enq_seen = enq;
      oss << ", raw_queue=" << raw->depth.load(std::memory_order_relaxed)
          << " (+" << delta << " packets since last tick)";
      if (delta == 0) {
        oss << " => 雷达/RTP 侧无新数据（链路或设备问题），不是软件堆积";
      } else {
        oss << " => 数据在进来但没有输出（ clogging：检查 CheckTimer 门控、队列是否已满、回调是否阻塞）";
      }
    }
    LogThrottled("publish_stall", oss.str(), 2000);
    stall_reported_ = true;
  } else if (stall_reported_) {
    std::ostringstream oss;
    oss << "[LivoxMon][INFO] point cloud output recovered after " << stall_peak_ms_
        << " ms stall, peak frame points="
        << points_max_.load(std::memory_order_relaxed) << ", max frame interval="
        << std::fixed << std::setprecision(1)
        << interval_max_ms_.load(std::memory_order_relaxed) << " ms";
    fprintf(stderr, "%s\n", oss.str().c_str());
    stall_reported_ = false;
    stall_peak_ms_ = 0;
  }
}

void QueueMonitor::LogThrottled(const std::string& key, const std::string& message,
                                uint64_t min_interval_ms) {
  const uint64_t now_ms = NowMs();
  auto it = last_warn_ms_.find(key);
  if (it != last_warn_ms_.end() && (now_ms - it->second) < min_interval_ms) {
    return;
  }
  last_warn_ms_[key] = now_ms;
  fprintf(stderr, "%s\n", message.c_str());
}

std::string QueueMonitor::DumpOneLine() const {
  const PublishSnapshot snap = GetPublishSnapshot();
  std::ostringstream oss;
  oss << "[LivoxMon] frames=" << snap.frames << " pts/frame avg=" << snap.points_avg
      << " max=" << snap.points_max << " interval_max=" << std::fixed
      << std::setprecision(1) << snap.interval_max_ms << "ms"
      << " cb_last=" << snap.callback_last_ms << "ms cb_max=" << snap.callback_max_ms
      << "ms stall=" << snap.stall_ms << "ms | ";

  std::vector<Gauge*> list;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    list = gauge_list_;
  }
  oss << "queues: ";
  for (Gauge* gauge : list) {
    if (gauge == nullptr) {
      continue;
    }
    oss << gauge->name << "=" << gauge->depth.load(std::memory_order_relaxed)
        << (gauge->capacity > 0 ? ("/" + std::to_string(gauge->capacity)) : "")
        << " ";
  }
  oss << "| skip: ";
  for (uint32_t i = 0; i < kSkipReasonCount; ++i) {
    oss << kSkipReasonName[i] << "="
        << skip_[i].load(std::memory_order_relaxed) << " ";
  }
  return oss.str();
}

std::string QueueMonitor::Dump() const {
  const PublishSnapshot snap = GetPublishSnapshot();
  std::ostringstream oss;
  oss << "==== Livox pipeline monitor ====\n";
  oss << "running=" << (IsRunning() ? 1 : 0) << " period=" << cfg_.period_ms
      << "ms stall_warn=" << cfg_.stall_warn_ms
      << "ms callback_warn=" << cfg_.callback_warn_ms << "ms\n";
  oss << "publish: frames=" << snap.frames << " points=" << snap.points_total
      << " pts/frame avg=" << snap.points_avg << " max=" << snap.points_max
      << " interval_max=" << std::fixed << std::setprecision(1)
      << snap.interval_max_ms << "ms callback_last=" << snap.callback_last_ms
      << "ms callback_max=" << snap.callback_max_ms << "ms stall=" << snap.stall_ms
      << "ms\n";
  oss << "skip_reason: ";
  for (uint32_t i = 0; i < kSkipReasonCount; ++i) {
    oss << kSkipReasonName[i] << "=" << skip_[i].load(std::memory_order_relaxed) << " ";
  }
  oss << "\n";
  oss << std::left << std::setw(28) << "queue" << std::right << std::setw(8) << "cap"
      << std::setw(10) << "depth" << std::setw(12) << "peak" << std::setw(12)
      << "enqueue" << std::setw(12) << "dequeue" << std::setw(10) << "drop"
      << std::setw(12) << "stale_ms" << "\n";

  std::vector<Gauge*> list;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    list = gauge_list_;
  }
  const uint64_t now_ms = NowMs();
  for (Gauge* gauge : list) {
    if (gauge == nullptr) {
      continue;
    }
    const uint64_t last = gauge->last_change_ms.load(std::memory_order_relaxed);
    const uint64_t stale = (last != 0 && now_ms > last) ? (now_ms - last) : 0;
    oss << std::left << std::setw(28) << gauge->name << std::right << std::setw(8)
        << (gauge->capacity > 0 ? std::to_string(gauge->capacity) : "-")
        << std::setw(10) << gauge->depth.load(std::memory_order_relaxed)
        << std::setw(12) << gauge->peak.load(std::memory_order_relaxed)
        << std::setw(12) << gauge->enqueue.load(std::memory_order_relaxed)
        << std::setw(12) << gauge->dequeue.load(std::memory_order_relaxed)
        << std::setw(10) << gauge->drop.load(std::memory_order_relaxed)
        << std::setw(12) << stale << "\n";
  }
  return oss.str();
}

}  // namespace livox_ros
