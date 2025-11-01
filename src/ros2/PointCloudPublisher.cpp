#include "PointCloudPublisher.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

PointCloudPublisher::PointCloudPublisher() 
    : Node("slam_pointcloud_publisher")
{
    // Create publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/slam/pointcloud", 10);
    
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/slam/features", 10);
    
    // Publish both Pose and PoseStamped
    pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam/camera_pose_stamped", 10);
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/slam/camera_pose", 10); 
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/slam/trajectory", 10);
    
    camera_path_.header.frame_id = "world";
    
    RCLCPP_INFO(this->get_logger(), "Publishing on topics:");
    RCLCPP_INFO(this->get_logger(), "  - /slam/pointcloud");
    RCLCPP_INFO(this->get_logger(), "  - /slam/camera_pose");
    RCLCPP_INFO(this->get_logger(), "  - /slam/trajectory");
    RCLCPP_INFO(this->get_logger(), "  - /slam/features");
}

sensor_msgs::msg::PointCloud2 PointCloudPublisher::createPointCloud2Msg(
    const std::vector<cv::Point3f>& points)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "world";
    
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    
    // Define fields: X, Y, Z, RGB
    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    
    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    
    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    
    sensor_msgs::msg::PointField field_rgb;
    field_rgb.name = "rgb";
    field_rgb.offset = 12;
    field_rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_rgb.count = 1;
    
    cloud_msg.fields.push_back(field_x);
    cloud_msg.fields.push_back(field_y);
    cloud_msg.fields.push_back(field_z);
    cloud_msg.fields.push_back(field_rgb);
    
    cloud_msg.point_step = 16; // 4 fields * 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
    
    // Fill point cloud data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");
    
    for (const auto& pt : points) {
        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;
        
        // Color based on height
        float height_ratio = std::min(std::max((pt.y + 2.0f) / 4.0f, 0.0f), 1.0f);
        
        uint8_t r = static_cast<uint8_t>(50 + 200 * (1.0f - height_ratio));
        uint8_t g = static_cast<uint8_t>(150);
        uint8_t b = static_cast<uint8_t>(50 + 200 * height_ratio);
        
        // Pack RGB into float
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        *iter_rgb = *reinterpret_cast<float*>(&rgb);
        
        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
    }
    
    return cloud_msg;
}

void PointCloudPublisher::publishPointCloud(const std::vector<cv::Point3f>& points)
{
    if (points.empty()) return;
    
    auto cloud_msg = createPointCloud2Msg(points);
    cloud_pub_->publish(cloud_msg);
}

void PointCloudPublisher::publishCameraPose(const cv::Mat& pose)
{
    if (pose.empty() || pose.rows != 4 || pose.cols != 4) return;
    
    // Create PoseStamped for trajectory
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = this->now();
    pose_stamped_msg.header.frame_id = "world";
    
    pose_stamped_msg.pose.position.x = pose.at<double>(0, 3);
    pose_stamped_msg.pose.position.y = pose.at<double>(1, 3);
    pose_stamped_msg.pose.position.z = pose.at<double>(2, 3);
    
    // Convert rotation matrix to quaternion
    cv::Mat R = pose(cv::Rect(0, 0, 3, 3));
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    
    double qw, qx, qy, qz;
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
        qy = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
        qz = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
    } else {
        if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
            qw = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
            qx = 0.25 * s;
            qy = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
            qz = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
        } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
            qw = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
            qx = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
            qy = 0.25 * s;
            qz = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
            qw = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
            qx = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
            qy = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
            qz = 0.25 * s;
        }
    }
    
    pose_stamped_msg.pose.orientation.w = qw;
    pose_stamped_msg.pose.orientation.x = qx;
    pose_stamped_msg.pose.orientation.y = qy;
    pose_stamped_msg.pose.orientation.z = qz;
    
    // Publish PoseStamped (for trajectory)
    pose_stamped_pub_->publish(pose_stamped_msg);
    
    // Publish plain Pose (for RViz2 Pose display)
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position = pose_stamped_msg.pose.position;
    pose_msg.orientation = pose_stamped_msg.pose.orientation;
    pose_pub_->publish(pose_msg);
    
    // Add to trajectory
    camera_path_.poses.push_back(pose_stamped_msg);
    camera_path_.header.stamp = this->now();
    path_pub_->publish(camera_path_);
}

void PointCloudPublisher::publishFeatureImage(const cv::Mat& image)
{
    if (image.empty()) return;
    
    auto cv_image = cv_bridge::CvImage();
    cv_image.header.stamp = this->now();
    cv_image.header.frame_id = "camera";
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = image;
    
    auto img_msg = cv_image.toImageMsg();
    image_pub_->publish(*img_msg);
}