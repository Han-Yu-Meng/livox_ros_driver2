#include "include/livox_driver2_api.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

std::mutex g_mtx;
pcl::PointCloud<pcl::PointXYZI> g_accumulated_cloud;
uint64_t g_last_save_time = 0;
int g_file_count = 0;
const uint64_t SAVE_INTERVAL_NS = 2000000000ULL; // 2 seconds in nanoseconds

void OnCustomMsg(const livox_ros::CustomMsg& msg) {
  std::lock_guard<std::mutex> lock(g_mtx);
  
  // Get current system time in nanoseconds
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  
  // Calculate delay (assuming msg.stamp is also in nanoseconds since epoch)
  double delay_ms = static_cast<double>(now_ns - msg.stamp) / 1000000.0;

  if (g_last_save_time == 0) {
    g_last_save_time = msg.stamp;
  }

  // for (const auto& p : msg.points) {
  //   pcl::PointXYZI pt;
  //   pt.x = p.x;
  //   pt.y = p.y;
  //   pt.z = p.z;
  //   pt.intensity = static_cast<float>(p.reflectivity);
  //   g_accumulated_cloud.push_back(pt);
  // }

  // if (msg.stamp - g_last_save_time >= SAVE_INTERVAL_NS) {
  //   if (!g_accumulated_cloud.empty()) {
  //     std::string filename = std::to_string(g_file_count++) + ".pcd";
  //     pcl::io::savePCDFileBinary(filename, g_accumulated_cloud);
  //     std::cout << "Saved " << filename << " with " << g_accumulated_cloud.size() << " points." << std::endl;
  //     g_accumulated_cloud.clear();
  //     g_last_save_time = msg.stamp;
  //   }
  // }

  static int count = 0;
  if (count++ % 10 == 0) {
    std::cout << "Received CustomMsg: "
              << "timestamp: " << msg.stamp << ", "
              << "point_num: " << msg.point_num 
              << " (Current accumulation: " << g_accumulated_cloud.size() << " points), "
              << "System Latency: " << std::fixed << std::setprecision(3) << delay_ms << " ms" << std::endl;
  }
}

void OnImuMsg(const livox_ros::ImuMsg& msg) {
  static int count = 0;
  if (count++ % 100 == 0) {
    std::cout << "Received ImuMsg (every 100th): "
              << "acc: [" << msg.linear_acceleration.x << ", " << msg.linear_acceleration.y << ", " << msg.linear_acceleration.z << "]" << std::endl;
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <config_json_path>" << std::endl;
    return -1;
  }

  std::string config_path = argv[1];
  auto driver = livox_ros::LivoxDriver::Create();

  driver->RegisterCustomMsgCallback(OnCustomMsg);
  driver->RegisterImuMsgCallback(OnImuMsg);

  if (!driver->LoadConfig(config_path)) {
    std::cerr << "Failed to load config: " << config_path << std::endl;
    return -1;
  }

  std::cout << "Starting Livox Driver..." << std::endl;
  if (!driver->Start()) {
    std::cerr << "Failed to start driver!" << std::endl;
    return -1;
  }

  std::cout << "Driver started. Press Ctrl+C to stop." << std::endl;
  
  // Keep alive for 30 seconds or until interrupted
  std::this_thread::sleep_for(std::chrono::seconds(30));

  std::cout << "Stopping Driver..." << std::endl;
  driver->Stop();

  return 0;
}
