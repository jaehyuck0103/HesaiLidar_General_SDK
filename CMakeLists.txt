cmake_minimum_required(VERSION 3.15)
project(PandarGeneralSDK)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_VERBOSE_MAKEFILE true)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE extern/asio/asio/include)
target_link_libraries(asio INTERFACE pthread)

add_library(
  ${PROJECT_NAME} SHARED
  src/pandar_gps_receiver.cpp src/pandar_lidar_receiver.cpp
  src/pandar_packet_parsers.cpp src/tcp_command_client.cpp src/pandar_utils.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include src/include)
target_link_libraries(${PROJECT_NAME} PRIVATE asio)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  find_package(PCL REQUIRED)

  add_executable(runSDK test/runSDK.cpp)
  target_include_directories(runSDK PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(runSDK PRIVATE ${PROJECT_NAME} ${PCL_LIBRARIES} pthread
                                       tbb asio)

  add_executable(sendPcap test/sendPcapPacket.cpp test/pcap_reader.cpp)
  target_link_libraries(sendPcap PRIVATE pcap asio)
endif()
