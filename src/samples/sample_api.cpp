#include "include/livox_driver2_api.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

void OnCustomMsg(const livox_ros::CustomMsg& msg) {
  std::cout << "Received CustomMsg: "
            << "timestamp: " << msg.stamp << ", "
            << "point_num: " << msg.point_num << std::endl;
  
  if (msg.point_num > 0) {
    std::cout << "First 3 points:" << std::endl;
    for (uint32_t i = 0; i < std::min(msg.point_num, 3u); ++i) {
      const auto& p = msg.points[i];
      std::cout << "  p[" << i << "]: x=" << std::fixed << std::setprecision(3) << p.x 
                << " y=" << p.y << " z=" << p.z << " reflectivity=" << (int)p.reflectivity << std::endl;
    }
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
