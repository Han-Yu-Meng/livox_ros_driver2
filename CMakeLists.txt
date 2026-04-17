cmake_minimum_required(VERSION 3.14)
project(livox_driver2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL       REQUIRED)
find_package(livox_msg REQUIRED)

# Find Livox SDK2
find_package(LivoxSDK REQUIRED)

# Core Driver Library
add_library(${PROJECT_NAME}_core STATIC
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
)

if(LIVOX_LIDAR_SDK_INCLUDE_DIR)
  target_include_directories(${PROJECT_NAME}_core PRIVATE ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
endif()

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ${LIVOX_LIDAR_SDK_LIBRARY}
  pthread
  std_msgs::std_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  livox_msg::livox_msg
  LivoxSDK::livox_lidar_sdk_static
)

# FINS node
fins_add_node(${PROJECT_NAME} SHARED 
  src/node/node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

fins_link_ros_dependencies(${PROJECT_NAME}
  std_msgs
  sensor_msgs
  pcl_conversions
)