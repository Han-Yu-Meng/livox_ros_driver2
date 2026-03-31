#include <fins/node.hpp>
#include "include/livox_driver2_api.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class LivoxDriverNode : public fins::Node {
public:
  void define() override {
    set_name("LivoxDriverNode");
    set_description("Livox Lidar driver node integrated with FINS framework.");
    set_category("Sensor>Lidar");

    // FINS Outputs
    register_output<livox_ros::ImuMsg>("imu");
    register_output<livox_ros::CustomMsg>("lidar");

    // Parameters
    register_parameter<std::string>("config_path", &LivoxDriverNode::on_config_path_changed, "MID360_config.json");
  }

  void initialize() override {
    driver_ = livox_ros::LivoxDriver::Create();
    
    driver_->RegisterCustomMsgCallback([this](const livox_ros::CustomMsg& msg) {
        this->on_custom_msg(msg);
    });
    
    driver_->RegisterImuMsgCallback([this](const livox_ros::ImuMsg& msg) {
        this->on_imu_msg(msg);
    });

    if (!config_path_.empty()) {
        load_and_start();
    }
  }

  void run() override {
    // Driver starts in load_and_start/initialize
  }

  void pause() override {
    if (driver_) {
        driver_->Stop();
    }
  }

  void reset() override {
    pause();
    load_and_start();
  }

  ~LivoxDriverNode() {
    if (driver_) {
        driver_->Stop();
    }
  }

private:
  void on_config_path_changed(const std::string& v) {
    if (config_path_ == v) return;
    config_path_ = v;
    logger->info("Config path changed to {}. Reloading...", v);
    load_and_start();
  }

  void load_and_start() {
    if (!driver_) return;
    driver_->Stop();
    if (!driver_->LoadConfig(config_path_)) {
      logger->error("Failed to load config: {}", config_path_);
      return;
    }
    if (!driver_->Start()) {
      logger->error("Failed to start livox driver!");
    } else {
      logger->info("Livox driver started with config: {}", config_path_);
    }
  }

  void on_custom_msg(const livox_ros::CustomMsg& msg) {
    // Publish via FINS
    send("lidar", msg, fins::now());
    
    uint64_t msg_ns = static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL + msg.header.stamp.nanosec;
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    double delay_ms = static_cast<double>(now_ns - msg_ns) / 1000000.0;
     
    /*
    // Maintain existing PCD saving logic for debug/utility
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Calculate delay using msg.header.stamp
    if (last_save_time_ == 0) {
      last_save_time_ = msg_ns;
    }

    for (const auto& p : msg.points) {
      pcl::PointXYZI pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = static_cast<float>(p.reflectivity);
      accumulated_cloud_.push_back(pt);
    }

    if (msg_ns - last_save_time_ >= 2000000000ULL) { // 2 seconds
      if (!accumulated_cloud_.empty()) {
        std::string filename = std::to_string(file_count_++) + ".pcd";
        pcl::io::savePCDFileBinary(filename, accumulated_cloud_);
        logger->info("Saved {} with {} points.", filename, accumulated_cloud_.size());
        accumulated_cloud_.clear();
        last_save_time_ = msg_ns;
      }
    }*/

    static int count = 0;
    if (count++ % 10 == 0) {
      logger->debug("Received CustomMsg: timestamp: {}.{}, point_num: {}, Latency: {} ms", 
                    msg.header.stamp.sec, msg.header.stamp.nanosec, msg.point_num, delay_ms);
    }
  }

  void on_imu_msg(const livox_ros::ImuMsg& msg) {
    send("imu", msg, fins::now());
    static int count = 0;
    if (count++ % 100 == 0) {
      logger->debug("Received ImuMsg (every 100th): acc: [{}, {}, {}]", 
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    }
  }

private:
  std::unique_ptr<livox_ros::LivoxDriver> driver_;
  std::string config_path_;
  
  std::mutex mtx_;
  pcl::PointCloud<pcl::PointXYZI> accumulated_cloud_;
  uint64_t last_save_time_ = 0;
  int file_count_ = 0;
};

EXPORT_NODE(LivoxDriverNode)
DEFINE_PLUGIN_ENTRY()
