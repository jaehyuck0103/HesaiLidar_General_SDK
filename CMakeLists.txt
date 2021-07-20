cmake_minimum_required(VERSION 3.15)
project(PandarGeneralSDK)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_VERBOSE_MAKEFILE true)

find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src/PandarGeneralRaw)

add_library(${PROJECT_NAME} SHARED src/pandarGeneral_sdk.cc
                                   src/tcp_command_client.cpp src/util.cpp)

target_include_directories(
  ${PROJECT_NAME} PRIVATE include src/PandarGeneralRaw/include
                          ${PCL_INCLUDE_DIRS} pthread)

target_link_libraries(${PROJECT_NAME} PandarGeneral ${PCL_IO_LIBRARIES} pcap
                      Eigen3::Eigen)

add_executable(PandarGeneralTest test/test.cc)

target_include_directories(
  PandarGeneralTest PRIVATE include src/PandarGeneralRaw/include
                            ${PCL_INCLUDE_DIRS})
target_link_libraries(PandarGeneralTest ${PROJECT_NAME} ${PCL_LIBRARIES}
                      pthread)
