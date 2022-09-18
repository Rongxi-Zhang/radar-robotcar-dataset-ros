cmake_minimum_required(VERSION 3.5)
project(radar_robotcar_dataset_ros2)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Added content
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets LinguistTools REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV 4.6.0 REQUIRED) #ros foxy default 4.2.0

set(dependencies
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
  "gps_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2_ros"
  "cv_bridge"
  Boost
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/gui
  ${PCL_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(TS_FILES gui/RobotCarPlayer_en_US.ts)
set(QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gui/image.qrc)
qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})
qt5_wrap_cpp(HEADER_SRC 
  include/${PROJECT_NAME}/robotcarplayer.h
  include/${PROJECT_NAME}/rclthread.hpp
)

add_executable(RobotCarPlayer
  src/main.cpp
  src/robotcarplayer.cpp
  src/robotcarplayer.ui
  src/rclthread.cpp
  ${TS_FILES}
  ${QRC_FILES}
  ${HEADER_SRC}
)

ament_target_dependencies(RobotCarPlayer
  ${dependencies}
)

target_link_libraries(RobotCarPlayer 
  Qt5::Widgets
  ${PCL_LIBRARIES}
)

install(TARGETS RobotCarPlayer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})

ament_package()