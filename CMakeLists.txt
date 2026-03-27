cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Note: PCL removed as it's no longer used for messaging
# find_package(PCL QUIET) 

# Find Livox SDK2
find_library(LIVOX_LIDAR_SDK_LIBRARY NAMES livox_lidar_sdk_shared livox_lidar_sdk_static HINTS /usr/local/lib)
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR NAMES "livox_lidar_api.h" "livox_lidar_def.h" HINTS /usr/local/include /usr/include)

# Core Driver Library
add_library(${PROJECT_NAME} SHARED
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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
)

if(LIVOX_LIDAR_SDK_INCLUDE_DIR)
  target_include_directories(${PROJECT_NAME} PRIVATE ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
endif()

target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  pthread
)

# Sample Program
add_executable(livox_driver2_sample src/samples/sample_api.cpp)
target_link_libraries(livox_driver2_sample ${PROJECT_NAME})
