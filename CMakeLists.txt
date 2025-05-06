cmake_minimum_required(VERSION 3.8)
project(hw1_kitti_projection)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the first executable
add_executable(kitti_publisher_node src/kitti_publisher_node.cpp)

ament_target_dependencies(kitti_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
)

target_link_libraries(kitti_publisher_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add the second executable
add_executable(lidar_to_image_projection_node src/lidar_to_image_projection_node.cpp)

ament_target_dependencies(lidar_to_image_projection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  OpenCV
)

target_link_libraries(lidar_to_image_projection_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS
  kitti_publisher_node
  lidar_to_image_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
