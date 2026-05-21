#include <fins/node.hpp>
#include "include/livox_driver2_api.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <atomic>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class LivoxDriverNode : public fins::Node {
public:
  void define() override {
    set_name("LivoxDriverNode");
    set_description("Livox Lidar driver node.");
    set_category("Driver");

    register_output<livox_ros::ImuMsg>("imu");
    register_output<livox_ros::CustomMsg>("lidar");
    register_output<sensor_msgs::msg::PointCloud2>("lidar_standard");

    register_parameter<std::string>("config_path", &LivoxDriverNode::on_config_path_changed, "/path/to/MID360_config.json");
    register_parameter<double>("publish_freq", &LivoxDriverNode::on_publish_freq_changed, 10.0);
    register_parameter<std::string>("frame_id", &LivoxDriverNode::on_frame_id_changed, "livox_frame");
  }

  void initialize() override {
    driver_ = livox_ros::LivoxDriver::Create();
    
    driver_->RegisterCustomMsgCallback([this](const livox_ros::CustomMsg& msg, uint8_t index) {
      this->on_custom_msg(msg);
    });
    
    driver_->RegisterImuMsgCallback([this](const livox_ros::ImuMsg& msg, uint8_t index) {
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
    update_driver_parameters();
    load_and_start();
  }

  void on_multi_topic_changed(int v) {
    if (multi_topic_ == v) return;
    multi_topic_ = v;
    logger->info("Multi-topic changed to {}. Updating...", v);
    update_driver_parameters();
  }

  void on_publish_freq_changed(double v) {
    if (publish_freq_ == v) return;
    publish_freq_ = v;
    logger->info("Publish frequency changed to {}. Updating...", v);
    update_driver_parameters();
  }

  void on_frame_id_changed(const std::string& v) {
    if (frame_id_ == v) return;
    frame_id_ = v;
    logger->info("Frame ID changed to {}. Updating...", v);
    update_driver_parameters();
  }

  void update_driver_parameters() {
    if (driver_) {
      driver_->SetParameters(multi_topic_, publish_freq_, frame_id_);
    }
  }

  void load_and_start() {
    if (!driver_) return;
    driver_->Stop();
    update_driver_parameters();
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
    uint64_t timestamp_ns = msg.timebase;
    if (!msg.points.empty()) {
      timestamp_ns += msg.points.back().offset_time;
    }
    
    if (required("lidar")) {
      send("lidar", msg, fins::from_seconds(timestamp_ns / 1e9));
    }

    if (required("lidar_standard")) {
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
      pcl_cloud.reserve(msg.points.size());
      for (const auto& p : msg.points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = static_cast<float>(p.reflectivity);
        pcl_cloud.push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 standard_msg;
      pcl::toROSMsg(pcl_cloud, standard_msg);
      standard_msg.header = msg.header;
      send("lidar_standard", standard_msg, fins::from_seconds(timestamp_ns / 1e9));
    }
  }

  void on_imu_msg(const livox_ros::ImuMsg& msg) {
    if (required("imu")) {
      send("imu", msg, fins::from_ros_time(msg.header.stamp));
    }
  }

private:
  std::unique_ptr<livox_ros::LivoxDriver> driver_;
  std::string config_path_;
  int multi_topic_ = 0;
  double publish_freq_ = 10.0;
  std::string frame_id_ = "livox_frame";
  
  std::mutex mtx_;
  pcl::PointCloud<pcl::PointXYZI> accumulated_cloud_;
  uint64_t last_save_time_ = 0;
  int file_count_ = 0;
};

class MultiLivoxDriverNode : public fins::Node {
public:
  void define() override {
    set_name("MultiLivoxDriverNode");
    set_description("Multi-Livox Lidar driver node.");
    set_category("Driver");

    for (int i = 1; i <= 2; ++i) {
      register_output<livox_ros::ImuMsg>("imu" + std::to_string(i));
      register_output<livox_ros::CustomMsg>("lidar" + std::to_string(i));
      register_output<sensor_msgs::msg::PointCloud2>("lidar_standard" + std::to_string(i));
    }

    register_parameter<std::string>("config_path", &MultiLivoxDriverNode::on_config_path_changed, "/path/to/MID360_config.json");
    register_parameter<double>("publish_freq", &MultiLivoxDriverNode::on_publish_freq_changed, 10.0);
    register_parameter<std::string>("frame_id", &MultiLivoxDriverNode::on_frame_id_changed, "livox_frame");
  }

  void initialize() override {
    driver_ = livox_ros::LivoxDriver::Create();
    
    driver_->RegisterCustomMsgCallback([this](const livox_ros::CustomMsg& msg, uint8_t index) {
      this->on_custom_msg(msg, index);
    });
    
    driver_->RegisterImuMsgCallback([this](const livox_ros::ImuMsg& msg, uint8_t index) {
      this->on_imu_msg(msg, index);
    });

    if (!config_path_.empty()) {
      load_and_start();
    }
  }

  void run() override {}

  void pause() override {
    if (driver_) {
      driver_->Stop();
    }
  }

  void reset() override {
    pause();
    load_and_start();
  }

  ~MultiLivoxDriverNode() {
    if (driver_) {
      driver_->Stop();
    }
  }

private:
  void on_config_path_changed(const std::string& v) {
    if (config_path_ == v) return;
    config_path_ = v;
    logger->info("Config path changed to {}. Reloading...", v);
    update_driver_parameters();
    load_and_start();
  }

  void on_publish_freq_changed(double v) {
    if (publish_freq_ == v) return;
    publish_freq_ = v;
    logger->info("Publish frequency changed to {}. Updating...", v);
    update_driver_parameters();
  }

  void on_frame_id_changed(const std::string& v) {
    if (frame_id_ == v) return;
    frame_id_ = v;
    logger->info("Frame ID changed to {}. Updating...", v);
    update_driver_parameters();
  }

  void update_driver_parameters() {
    if (driver_) {
      driver_->SetParameters(0, publish_freq_, frame_id_);
    }
  }

  void load_and_start() {
    if (!driver_) return;
    driver_->Stop();
    update_driver_parameters();
    if (!driver_->LoadConfig(config_path_)) {
      logger->error("Failed to load config: {}", config_path_);
      return;
    }
    if (!driver_->Start()) {
      logger->error("Failed to start multi-livox driver!");
    } else {
      logger->info("Multi-livox driver started with config: {}", config_path_);
    }
  }

  void on_custom_msg(const livox_ros::CustomMsg& msg, uint8_t index) {
    int id = index + 1;
    std::string suffix = std::to_string(id);
    
    uint64_t timestamp_ns = msg.timebase;
    if (!msg.points.empty()) {
      timestamp_ns += msg.points.back().offset_time;
    }
    
    if (required("lidar" + suffix)) {
      send("lidar" + suffix, msg, fins::from_seconds(timestamp_ns / 1e9));
    }

    if (required("lidar_standard" + suffix)) {
      pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
      pcl_cloud.reserve(msg.points.size());
      for (const auto& p : msg.points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = static_cast<float>(p.reflectivity);
        pcl_cloud.push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 standard_msg;
      pcl::toROSMsg(pcl_cloud, standard_msg);
      standard_msg.header = msg.header;
      send("lidar_standard" + suffix, standard_msg, fins::from_seconds(timestamp_ns / 1e9));
    }
  }

  void on_imu_msg(const livox_ros::ImuMsg& msg, uint8_t index) {
    int id = index + 1;
    std::string suffix = std::to_string(id);
    if (required("imu" + suffix)) {
      send("imu" + suffix, msg, fins::from_ros_time(msg.header.stamp));
    }
  }

private:
  std::unique_ptr<livox_ros::LivoxDriver> driver_;
  std::string config_path_;
  double publish_freq_ = 10.0;
  std::string frame_id_ = "livox_frame";
};

EXPORT_NODE(LivoxDriverNode)
EXPORT_NODE(MultiLivoxDriverNode)
DEFINE_PLUGIN_ENTRY(fins::STATELESS)