// File Related
#include <sstream>
#include <fstream>
#include <iterator>

// ROS2 Related
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL Related
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV Related
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Math & Matrix Related
#include <Eigen/Dense>

class LidarToImageProjector : public rclcpp::Node {
public:
    LidarToImageProjector() : Node("lidar_to_image_projector") {
        loadCalibrationData();

        // Make 2 subscribers that can subscribe LiDAR & Image data
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kitti/points", 10, std::bind(&LidarToImageProjector::lidarCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/kitti/image", 10, std::bind(&LidarToImageProjector::imageCallback, this, std::placeholders::_1));

        // Make 2 publishers that can publish Image data with projected LiDAR data
        image_with_lidar_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);

        // Log publish start msg at the first start
        RCLCPP_INFO(this->get_logger(), "Subscribed to /kitti/points and /kitti/image");
        RCLCPP_INFO(this->get_logger(), "Publishing to /projected_image and /projected_cloud");
    }

private:
    /* Load calculated calibration projection_matrix_ for transforming LiDAR data */
    void loadCalibrationData() {
        // Set folder path of vel_to_cam & cam_to_cam (.txt) data
        std::ifstream velo_to_cam_file("/home/dayeon/ros2_ws/data/2011_09_26/calib_velo_to_cam.txt");
        std::ifstream cam_to_cam_file("/home/dayeon/ros2_ws/data/2011_09_26/calib_cam_to_cam.txt");

        //! Sensor Calibration Formula
        //! P_rect_02 [3x4] * R_rect_00 [4x4] * Tr_velo_to_cam [4x4] * (x,y,z,1) [4x1]

        //* 1. Find Tr_velo_to_cam
        Eigen::Matrix4f Tr_velo_to_cam_ = Eigen::Matrix4f::Identity(); // 4x4 matrix of floats (Tr_velo_to_cam_)
        std::string line;              // String variable
        std::vector<float> rot, trans; // Vector used to store rotation and translation data

        // Find Tr_velo_to_Cam [4x4]
        while (std::getline(velo_to_cam_file, line)) {
            if (line.rfind("R:", 0) == 0) {                                                    // Check if the line starts with "R:" (rotation matrix)
                std::istringstream ss(line.substr(3));                                         // Erase the "R:" part
                rot.assign(std::istream_iterator<float>(ss), std::istream_iterator<float>());  // Convert string -> float and save to rot vector
            }
            else if (line.rfind("T:", 0) == 0) {                                                // Check if the line starts with "T:" (translation matrix)
                std::istringstream ss(line.substr(3));                                          // Erase the "T:" part
                trans.assign(std::istream_iterator<float>(ss), std::istream_iterator<float>()); // Convert string -> float and save to trans vector
            }
        }

        // Reshape rotation from Vector [9x1] -> Eigen Matrix [3x3] and translation to Vector [3x1] -> Eigen Vector [3x1]
        if (rot.size() == 9 && trans.size() == 3) {
            Eigen::Matrix3f R;            // Create a 3x3 rotation matrix
            for (int i = 0; i < 9; ++i) { // Map 1D rotation values to 2D matrix: row = i / 3, col = i % 3
                R(i / 3, i % 3) = rot[i];
            }
            Eigen::Vector3f T(trans[0], trans[1], trans[2]); // Create a 3x1 translation matrix

            Tr_velo_to_cam_.block<3,3>(0,0) = R; // Insert R into the top-left 3x3 block of the 4x4 matrix
            Tr_velo_to_cam_.block<3,1>(0,3) = T; // Insert T into the top-right 3x1 block of the 4x4 matrix
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid rotation or translation data size."); // If size does not match, log error msg
        }

        //* 2. Find R_rect_00 [4x4] & P_rect_02 [3x4]
        Eigen::Matrix4f R_rect_00_ = Eigen::Matrix4f::Identity(); // 4x4 matrix of floats (R_Rect_00_)
        Eigen::Matrix<float, 3, 4> P_rect_02_;                    // 3x4 matrix of floats (P_rect_02_)
        std::vector<float> r_rect, p_rect;                        // Vector used to store r_rect and p_rect data

        while (std::getline(cam_to_cam_file, line)) {
            if (line.rfind("R_rect_00:", 0) == 0) {                                              // Check if the line starts with "R_rect_00:"
                std::istringstream ss(line.substr(11));                                          // Erase the "R_rect_00:" part
                r_rect.assign(std::istream_iterator<float>(ss), std::istream_iterator<float>()); // Convert string -> float and save to r_rect vector
            }
            else if (line.rfind("P_rect_02:", 0) == 0) {                                         // Check if the line starts with "P_rect_02:"
                std::istringstream ss(line.substr(11));                                          // Erase the "P_rect_02:" part
                p_rect.assign(std::istream_iterator<float>(ss), std::istream_iterator<float>()); // Convert string -> float and save to p_rect vector
            }
        }

        // Reshape R_rect_00_ from Vector [9x1] -> Eigen Matrix [3x3]
        if (r_rect.size() == 9) {
            Eigen::Matrix3f R_r;               // Create a 3x3 rotation matrix
            for (int i = 0; i < 9; ++i) {
                R_r(i / 3, i % 3) = r_rect[i]; // Map 1D r_rect values to 2D matrix: row = i / 3, col = i % 3
            }
            R_rect_00_.block<3,3>(0,0) = R_r;  // Insert R_r into the top-left 3x3 block of the 4x4 matrix
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid R_rect_00 size"); // If size does not match, log error msg
        }

        // Reshape P_rect_02_ from Vector [12x1] -> Eigen Matrix [3x4]
        if (p_rect.size() == 12) {
            for (int i = 0; i < 12; ++i) {
                P_rect_02_(i / 4, i % 4) = p_rect[i]; // Map 1D p_rect values to 2D matrix: row = i / 4, col = i % 4
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid P_rect_02 size"); // If size does not match, log error msg
        }

        //* 3. Projection Calculation
        projection_matrix_ = P_rect_02_ * R_rect_00_ * Tr_velo_to_cam_;
    }

    /* Create a bridge for conversion btw ROS2 Image -> OpenCV Image */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> lidar_cloud; // Create a lidar_cloud object that can store x, y, z, and intensity values
        pcl::fromROSMsg(*msg, lidar_cloud);          // Convert ROS msg format (msg) -> PCL point cloud(lidar_cloud)

        if (image_.empty()) return; // If camera image hasn't arrived yet, exit function

        std::vector<cv::Point2f> projected_points;           // Create a vector to store x, y points projected on to the camera image
        projectLidarToCamera(lidar_cloud, projected_points); // Function that projects LiDAR point cloud data and map them to 2d image coordinates

        cv::Mat image_with_lidar = image_.clone(); // Copy image to draw LiDAR points on it
        for (const auto& pt : projected_points) {
            cv::circle(image_with_lidar, pt, 2, cv::Scalar(0, 255, 0), -1); // Draw projected LiDAR points in green on the image
        }
        RCLCPP_INFO(this->get_logger(), "Projection completed");

        std_msgs::msg::Header header;     // Create header for ROS Message
        cv_bridge::CvImage out_img(header, "bgr8", image_with_lidar);      // Create a bridge for conversion btw OpenCV -> ROS2 Image
        sensor_msgs::msg::Image::SharedPtr img_msg = out_img.toImageMsg(); // Convert OpenCV image -> ROS2 Image
        image_with_lidar_pub_->publish(*img_msg);                          // Publish the message to the ROS2 topic (/projected_image)
    }

    void projectLidarToCamera(const pcl::PointCloud<pcl::PointXYZI>& lidar_cloud, std::vector<cv::Point2f>& projected_points) {
        for (const auto& point : lidar_cloud.points) {
            Eigen::Vector4f lidar_point(point.x, point.y, point.z, 1.0f);     // Convert 3D LiDAR point cloud data to homogeneous coordinate vector
            Eigen::Vector3f proj = projection_matrix_ * lidar_point; // Apply projection matrix to map the 3D point in to camera's 2D image coordinate system

            float z = proj(2);    // Check if the points are in front of the camera
            if (z <= 0) continue; // z <= 0 means back of the camera

            // Perspective division to convert the homogeneous coordinates into 2D pixel coordinates
            // The matrix has to be divided by the scalar z to convert (x, y, z) to (u, v, 1)
            float x = proj(0) / z;
            float y = proj(1) / z;

            // Use points that are in the range within the image boundaries (width and height)
            if (x >= 0 && x < image_.cols && y >= 0 && y < image_.rows) {
                projected_points.push_back(cv::Point2f(x, y)); // Store the valid 2D image point into the output projected_points vector
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;        // ROS2 subscriber for LiDAR point cloud data
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;              // ROS2 subscriber for camera image data
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_with_lidar_pub_;      // ROS2 publisher for image data with projected LiDAR data
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_cloud_pub_; // ROS2 publisher for projectedLiDAR Data

    cv::Mat image_;                                                // Latest received camera image, stored as an OpenCV matrix
    Eigen::Matrix4f Tr_velo_to_cam_ = Eigen::Matrix4f::Identity(); // Calibration matrix to transform LiDAR point cloud frame to camera frame
    Eigen::Matrix4f R_rect_00_ = Eigen::Matrix4f::Identity();      // Rectification ,atrix for the camera
    Eigen::Matrix<float, 3, 4> P_rect_02_;                         // Camera projection matrix
    Eigen::Matrix<float, 3, 4> projection_matrix_;                 // Final projection matrix used to project LiDAR point cloud on to the image plane

    pcl::PointCloud<pcl::PointXYZI> projected_cloud_;              // Container to store the LiDAR point cloud after being projected on to the image plane
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToImageProjector>());
    rclcpp::shutdown();
    return 0;
}