#ifndef POINTCLOUD_PUBLISHER_HPP
#define POINTCLOUD_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp> 
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

class PointCloudPublisher : public rclcpp::Node {
public:
    PointCloudPublisher();
    
    void publishPointCloud(const std::vector<cv::Point3f>& points);
    void publishCameraPose(const cv::Mat& pose);
    void publishFeatureImage(const cv::Mat& image);
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;  // Keep for Path
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;  // ✅ ADD: For RViz2 display
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    nav_msgs::msg::Path camera_path_;
    
    sensor_msgs::msg::PointCloud2 createPointCloud2Msg(
        const std::vector<cv::Point3f>& points);
};

#endif
