cmake_minimum_required(VERSION 3.15)
project(PandarSDK)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_VERBOSE_MAKEFILE true)

include(FetchContent)
# Fetch ASIO
FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-19-2)
FetchContent_MakeAvailable(asio)

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(asio INTERFACE pthread)

# Fetch nlohmann_json
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY
    https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent.git
  GIT_TAG v3.10.0)
FetchContent_MakeAvailable(nlohmann_json)

# Pandar Library
add_library(
  ${PROJECT_NAME}
  src/pandar_gps_receiver.cpp src/pandar_lidar_receiver.cpp
  src/pandar_packet_parsers.cpp src/tcp_command_client.cpp src/pandar_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE nlohmann_json::nlohmann_json
  PUBLIC asio)

# Test
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  find_package(PCL REQUIRED)

  add_executable(runSDK test/runSDK.cpp)
  target_include_directories(runSDK PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(runSDK PRIVATE ${PROJECT_NAME} ${PCL_LIBRARIES} tbb)

  add_executable(sendPcap test/sendPcapPacket.cpp test/pcap_reader.cpp)
  target_link_libraries(sendPcap PRIVATE pcap asio)
endif()
