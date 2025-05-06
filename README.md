# hw1_kitti_projection
This repo is a project for implementing a ROS2 node class that subscribes camera and LiDAR topics, and then projects the point cloud data onto the RGB image, later publishing the result by RViz2.

## ? My Environment
|Settings|Version|
|------|---|
|OS|Ubuntu 22.04.5 LTS|
|Language|C++|
|ROS2|Humble|
|PCL|1.12.1|
|OpenCV|4.5.4|
|Eigen|3.4.0|

## ? Before Start
### ?KITTI Dataset Download
- https://www.cvlibs.net/datasets/kitti/raw_data.php

### ? Used Data
- 2011_09_26_drive_0015 (1.2GB) Data

    - [synced+rectified data]
    - [calibration]

## ? Setup
### 1. Download or clone this repo under the {ROS_Workspace/src} folder.
~~~ bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/DayeonSeo416/hw1_kitti_projection.git
~~~

### 2. After downloading the dataset, create the folder named "data" in the workspace, and extract the dataaset in the data folder.
~~~ bash
ex) /home/USER/ros2_ws/data/2011_09_26/2011_09_26_drive_0015_sync/...
~~~
#### Note that the folder tree should look like this :
~~~ text
.ros2_ws
戍 build
戍 data
戍式式式式式戍 2011_09_26
戍式式式式式式式式式式式式式式式式式戍 2011_09_26_drive_0015_sync
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 image_00
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 image_01
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 image_02
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 image_03
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 oxts
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 velodyne_points
戍式式式式式式式式式式式式式式式式式戍 calib_cam_to_cam.txt
戍式式式式式式式式式式式式式式式式式戍 calib_imu_to_velo.txt
戍式式式式式式式式式式式式式式式式式戍 calib_velo_to_cam.txt
戍 install
戍 log
戍 src
戍式式式式式戍 hw1_kitti_projection
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 CMakeLists.txt
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 default.rviz
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 package.xml
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 README.md
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 src
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 kitti_publisher_node.cpp
戍式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式戍 lidar_to_image_projection_node.cpp
~~~

### 3. If you want to use the other dataset, modify the path in each of the cpp file.
~~~ bash
//* Modify the path of the dataset *//

- kitti_publisher_node.cpp

lidar_dir_="/home/USER/ros2_ws/data/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data";
image_dir_="/home/USER/ros2_ws/data/2011_09_26/2011_09_26_drive_0015_sync/image_02/data";

- lidar_to_image_projection_node.cpp

std::ifstream velo_to_cam_file("/home/dayeon/ros2_ws/data/2011_09_26/calib_velo_to_cam.txt");
std::ifstream cam_to_cam_file("/home/dayeon/ros2_ws/data/2011_09_26/calib_cam_to_cam.txt");
~~~

## ? How to Build
~~~ bash
$ cd ~/ros2_ws
$ colcon build --packages-select hw1_kitti_projection

$ source install/setup.bash

$ ros2 run hw1_kitti_projection kitti_publisher_node
$ ros2 run hw1_kitti_projection lidar_to_image_projection_node
$ rviz2
~~~

## ? Visualize RViz2
You could import the "default.rviz" file into the Rviz2.

![譫籀](/src/hw1_kitti_projection/RVIZ2_image.png)

