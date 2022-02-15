cmake_minimum_required(VERSION 3.17)
project(PandarSDK)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_VERBOSE_MAKEFILE true)

include(FetchContent)

# Fetch ASIO
FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-21-0)
FetchContent_MakeAvailable(asio)

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(asio INTERFACE pthread)

# Fetch nlohmann_json
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY
    https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent.git
  GIT_TAG v3.10.5)
FetchContent_MakeAvailable(nlohmann_json)

# Fetch spdlog
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.9.2)
FetchContent_MakeAvailable(spdlog)

# Fetch Eigen
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0)
FetchContent_MakeAvailable(Eigen)

# Pandar Library
add_library(
  ${PROJECT_NAME}
  src/pandar_gps_receiver.cpp src/pandar_lidar_receiver.cpp
  src/pandar_packet_parsers.cpp src/tcp_command_client.cpp src/pandar_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE nlohmann_json::nlohmann_json
  PUBLIC asio spdlog::spdlog Eigen3::Eigen)

# Test
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  find_package(PCL REQUIRED)

  add_executable(runSDK test/runSDK.cpp)
  target_include_directories(runSDK PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(runSDK PRIVATE ${PROJECT_NAME} ${PCL_LIBRARIES} tbb)

  add_executable(sendPcap test/sendPcapPacket.cpp test/pcap_reader.cpp)
  target_link_libraries(sendPcap PRIVATE pcap asio)
endif()
