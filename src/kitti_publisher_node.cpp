// File Related
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <filesystem>

// ROS2 Related
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL Related
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV Related
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

class KittiPublisherNode : public rclcpp::Node {
    public:
        KittiPublisherNode() : Node("kitti_publisher_node"), file_index_(0) {
            // Set folder path of LiDAR(.bin) and Image(.png) data
            lidar_dir_ = "/home/dayeon/ros2_ws/data/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data";
            image_dir_ = "/home/dayeon/ros2_ws/data/2011_09_26/2011_09_26_drive_0015_sync/image_02/data";

            // Make 2 publishers that can publish LiDAR & Image data
            lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/kitti/points", 10);
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kitti/image", 10);

            // Loop through LiDAR & image directories -> Saving all (.bin) & (.png) file path
            for (const auto& f : fs::directory_iterator(lidar_dir_)) {
                if (f.path().extension() == ".bin") {
                    lidar_files_.push_back(f.path().string());}
            }
            for (const auto& f : fs::directory_iterator(image_dir_)) {
                if (f.path().extension() == ".png") {
                    image_files_.push_back(f.path().string());}
            }

            // Sort all files by filename by ascending order
            std::sort(lidar_files_.begin(), lidar_files_.end());
            std::sort(image_files_.begin(), image_files_.end());

            // Publish LiDAR and image data every 100ms(0.1s)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&KittiPublisherNode::timerCallback, this) // Set a timer that calls 'timerCallback()' func every 100ms(0.1s)
            );

            RCLCPP_INFO(this->get_logger(), "Kitti Publisher Node Started."); // Log publish start msg at the first start
        }

    private:
        /* Stop publishing and log publish finish msg if we reached the end of the files */
        void timerCallback() {
            if (file_index_ >= lidar_files_.size() || file_index_ >= image_files_.size()) {
                RCLCPP_INFO(this->get_logger(), "Finish Publishing all Files.");
                rclcpp::shutdown(); // Shutdown ROS2 node
                return;             // Exit function
            }
            // Publish the current (.bin) & (.png) files by calling 2 funcs -- publishLidar & publishImage
            publishLidar(lidar_files_[file_index_]);
            publishImage(image_files_[file_index_]);

            file_index_++; // Increase file index
        }

        /* Read LiDAR Data from file path -> PointCloudXYZI -> Convert to ROS2 Message -> Publish */
        void publishLidar(const std::string& file_path) {
            std::ifstream input(file_path, std::ios::binary); // Open the (.bin) file in a binary mode
            if (!input.good()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read LiDAR File: %s", file_path.c_str()); // If unreadable, log error msg
                return;
            }

            // Initialize an empty point cloud container that can store LiDAR points with x, y, z, and intensity fields
            pcl::PointCloud<pcl::PointXYZI> cloud;
            input.seekg(0, std::ios::beg);

            // Read each point with 4 values (x, y, z, intensity) and add it into point cloud container.
            for (int i = 0; input.good() && !input.eof(); i++) {
                pcl::PointXYZI point;                                 // Create a point object that can store x, y, z, and intensity values
                input.read((char *) &point.x, 3*sizeof(float));       // Read x, y, z coords at once (3 * 4byte = 12byte)
                input.read((char *) &point.intensity, sizeof(float)); // Read intensity
                cloud.push_back(point);                               // Add the point to point cloud container
            }

            sensor_msgs::msg::PointCloud2 msg; // Create a PointCloud2 message object used in ROS2
            pcl::toROSMsg(cloud, msg);         // Convert PCL point cloud(cloud) -> ROS msg format (msg)
            msg.header.frame_id = "base_link"; // Set frame_id to base_link
            msg.header.stamp = this->now();    // Set current time as the timestamp of the message
            lidar_pub_->publish(msg);          // Publish the message to the ROS2 topic (/kitti/points)

            RCLCPP_INFO(this->get_logger(), "Published LiDAR data from file: %s", file_path.c_str()); // Log publish msg while publishing
        }

        /* Read Image Data from file path -> OpenCV -> Convert to ROS2 Message -> Publish */
        void publishImage(const std::string& file_path) {
            cv::Mat img = cv::imread(file_path, cv::IMREAD_COLOR);
            if (img.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read Image File: %s", file_path.c_str()); // If unreadable, log error msg
                return;
            }
            std_msgs::msg::Header header;                                 // Create header for ROS Message
            header.frame_id = "camera";                                   // Set frame_id to camera
            header.stamp = this->now();                                   // Set current time as the timestamp of the message
            cv_bridge::CvImage cv_img(header, "bgr8", img);               // Create a bridge for conversion btw OpenCV -> ROS2 Image
            sensor_msgs::msg::Image::SharedPtr msg = cv_img.toImageMsg(); // Convert OpenCV image -> ROS2 Image
            image_pub_->publish(*msg);                                    // Publish the message to the ROS2 topic (/kitti/image)

            RCLCPP_INFO(this->get_logger(), "Published Image from file: %s", file_path.c_str()); // Log publish msg while publishing
            RCLCPP_INFO(this->get_logger(), "----------------------"); // contour
        }

        std::vector<std::string> lidar_files_, image_files_; // Vector storing multiple fields of file paths of LiDAR and image files
        std::string lidar_dir_, image_dir_;                  // String storing file path of LiDAR and image files
        size_t file_index_;                                  // Idx of the current file to be published

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_; // ROS2 publisher for LiDAR Data
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;       // ROS2 publisher for Image Data
        rclcpp::TimerBase::SharedPtr timer_;                                    // Timer to call the callback func 'timerCallback'
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
