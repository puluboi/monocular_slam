#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <cmath>
#include "PointCloudPublisher.hpp"

using namespace std::chrono_literals;

class TestSLAMNode : public rclcpp::Node {
public:
    TestSLAMNode() : Node("test_slam_node"), angle_(0.0) {
        publisher_ = std::make_shared<PointCloudPublisher>();
        
        // Create timer to publish test data at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TestSLAMNode::publishTestData, this));
        
        RCLCPP_INFO(this->get_logger(), "Test SLAM node started - publishing demo data!");
        
    }

private:
    void publishTestData() {
        // Create a rotating point cloud (circle pattern)
        std::vector<cv::Point3f> test_points;
        
        for (int i = 0; i < 100; ++i) {
            float t = i * 2.0f * M_PI / 100.0f;
            float x = 2.0f * std::cos(t + angle_);
            float y = 0.5f * std::sin(3.0f * t);
            float z = 2.0f * std::sin(t + angle_);
            test_points.push_back(cv::Point3f(x, y, z));
        }
        
      
        
        publisher_->publishPointCloud();
        
        // Create camera pose (rotating around origin)
        cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
        pose.at<double>(0, 3) = 3.0 * std::cos(angle_);
        pose.at<double>(1, 3) = 0.5;
        pose.at<double>(2, 3) = 3.0 * std::sin(angle_);
        publisher_->publishCameraPose(pose);
        
        // Create test feature image
        cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::circle(test_image, cv::Point(320, 240), 50, cv::Scalar(0, 255, 0), -1);
        cv::putText(test_image, "Test SLAM Data", cv::Point(200, 240), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        publisher_->publishFeatureImage(test_image);
        
        angle_ += 0.05;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Publishing test point cloud: %zu points", test_points.size());
    }

    std::shared_ptr<PointCloudPublisher> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float angle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestSLAMNode>());
    rclcpp::shutdown();
    return 0;
}
