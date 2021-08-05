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

add_library(
  ${PROJECT_NAME} SHARED
  src/pandarGeneral_sdk.cpp
  src/input.cpp
  src/pandarGeneral_internal.cpp
  src/tcp_command_client.cpp
  src/util.cpp
  src/pcap_reader.cpp
  src/pandar40P.cpp
  src/pandar64.cpp
  src/pandarQT.cpp
  src/pandarXT.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include src/include
                                                   ${PCL_INCLUDE_DIRS} pthread)
target_link_libraries(${PROJECT_NAME} ${PCL_IO_LIBRARIES} pcap Eigen3::Eigen)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  add_executable(PandarGeneralTest test/test.cc)
  target_include_directories(PandarGeneralTest PRIVATE include src/include
                                                       ${PCL_INCLUDE_DIRS})
  target_link_libraries(PandarGeneralTest ${PROJECT_NAME} ${PCL_LIBRARIES}
                        pthread)
endif()
