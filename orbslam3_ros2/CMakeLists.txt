cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)


include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
target_link_libraries(mono ${OpenCV_LIBS})
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
target_link_libraries(rgbd ${OpenCV_LIBS})
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
target_link_libraries(stereo ${OpenCV_LIBS})
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
target_link_libraries(stereo-inertial ${OpenCV_LIBS})
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono-inertial
  src/mono-inertial/mono-inertial.cpp
  src/mono-inertial/mono-inertial-node.cpp
)
target_link_libraries(mono-inertial ${OpenCV_LIBS})
ament_target_dependencies(mono-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono-inertial2
  src/mono-inertial2/mono-inertial2.cpp
  src/mono-inertial2/mono-inertial-node2.cpp
)
target_link_libraries(mono-inertial2 ${OpenCV_LIBS})
ament_target_dependencies(mono-inertial2 rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono-inertial3
  src/mono-inertial3/mono-inertial3.cpp
  src/mono-inertial3/mono-inertial-node3.cpp
)
target_link_libraries(mono-inertial3 ${OpenCV_LIBS})
ament_target_dependencies(mono-inertial3 rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono-inertial4
  src/mono-inertial4/mono-inertial4.cpp
)
target_link_libraries(mono-inertial4 ${OpenCV_LIBS})
ament_target_dependencies(mono-inertial4 rclcpp sensor_msgs std_msgs nav_msgs tf2_ros tf2_geometry_msgs cv_bridge ORB_SLAM3 Pangolin)


install(TARGETS mono rgbd stereo stereo-inertial mono-inertial mono-inertial2 mono-inertial3 mono-inertial4
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()


