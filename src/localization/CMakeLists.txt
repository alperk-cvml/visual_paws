cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find ROS 2 dependencies ─────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Find ORB-SLAM3 dependencies ─────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin QUIET)  # Optional — only needed if viewer is enabled

# ── ORB-SLAM3 library (built at /opt/ORB_SLAM3) ─────────────────────────
set(ORB_SLAM3_ROOT "/opt/ORB_SLAM3")
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
)

# ORB-SLAM3 shared library
find_library(ORB_SLAM3_LIB ORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT}/lib
  NO_DEFAULT_PATH
)
if(NOT ORB_SLAM3_LIB)
  message(FATAL_ERROR "Could not find libORB_SLAM3.so in ${ORB_SLAM3_ROOT}/lib")
endif()

# DBoW2 and g2o (bundled with ORB-SLAM3)
find_library(DBOW2_LIB DBoW2
  PATHS ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib
  NO_DEFAULT_PATH
)
find_library(G2O_LIB g2o
  PATHS ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib
  NO_DEFAULT_PATH
)

# ── Build orbslam3_rgbd_node ─────────────────────────────────────────────
add_executable(orbslam3_rgbd_node
  src/orbslam3_rgbd_node.cpp
)

target_include_directories(orbslam3_rgbd_node PRIVATE
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(orbslam3_rgbd_node
  ${ORB_SLAM3_LIB}
  ${DBOW2_LIB}
  ${G2O_LIB}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

if(Pangolin_FOUND)
  target_link_libraries(orbslam3_rgbd_node ${Pangolin_LIBRARIES})
  target_include_directories(orbslam3_rgbd_node PRIVATE ${Pangolin_INCLUDE_DIRS})
endif()

ament_target_dependencies(orbslam3_rgbd_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  message_filters
  ament_index_cpp
)

# ── Install ──────────────────────────────────────────────────────────────
install(TARGETS orbslam3_rgbd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
