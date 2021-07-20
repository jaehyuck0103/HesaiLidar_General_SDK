cmake_minimum_required(VERSION 3.15)
project(PandarGeneral)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")

find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} src/input.cc src/pandarGeneral_internal.cc
                            src/pandarGeneral.cc src/pcap_reader.cpp)
target_include_directories(
  ${PROJECT_NAME} PRIVATE . include include/pandarGeneral ${Boost_INCLUDE_DIRS}
                          ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PCL_IO_LIBRARIES}
                      Eigen3::Eigen boost_thread)
