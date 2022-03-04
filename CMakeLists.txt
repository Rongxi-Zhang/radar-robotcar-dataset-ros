cmake_minimum_required(VERSION 2.8.3)
project(radar_robotcar_dataset_ros)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_models
  image_transport
  pcl_ros
  pcl_conversions
  message_generation
)

set_target_properties(${TARGET} PROPERTIES
COMPILE_FLAGS "-pthread"
LINK_FLAGS "-pthread")

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV  not found.")
endif()
MESSAGE(STATUS "OPENCV VERSION: ${OpenCV_VERSION}")

find_package(Eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3  not found.")
endif()
MESSAGE(status "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package(PCL REQUIRED)
MESSAGE(status "PCL_INCLUDE_DIR: ${PCL_INCLUDE_DIR}")


include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  Gps.msg
  Ins.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES oxford_image
  CATKINDEPENDS std_msgs roscpp
)

add_library(${PROJECT_NAME} SHARED
  src/oxford_sensor.cpp
  include/oxford_sensor.h
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CERES_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${PCL_LIBRARIES}
  )

add_executable(oxford_sensors src/OxFord_Sensors.cpp)
target_link_libraries(oxford_sensors ${PROJECT_NAME})
