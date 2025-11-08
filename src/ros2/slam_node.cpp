#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include "PointCloudPublisher.hpp"
#include "estimator/featureTracker.hpp" 

int main(int argc, char** argv)
{   
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Initialize the ros publiser
    auto ros_publisher = std::make_shared<PointCloudPublisher>();
    
    RCLCPP_INFO(ros_publisher->get_logger(), "Initializing SLAM system...");
    
    // Initialize the feature tracker
    featureTracker* tracker = nullptr;
    try {

        tracker = new featureTracker();
        tracker->setROSPublisher(ros_publisher);
        RCLCPP_INFO(ros_publisher->get_logger(), "Feature tracker initialized!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_publisher->get_logger(), "Failed: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(ros_publisher->get_logger(), "SLAM running! Topics:");
    RCLCPP_INFO(ros_publisher->get_logger(), "  /slam/pointcloud");
    RCLCPP_INFO(ros_publisher->get_logger(), "  /slam/camera_pose");
    RCLCPP_INFO(ros_publisher->get_logger(), "  /slam/trajectory");
    
    // Run orbTest() in loop
    std::atomic<bool> running(true);
    std::thread slam_thread([&]() {
        while (running && rclcpp::ok()) {
            try {
                tracker->orbTest();  
                
                // Access map points if available
                if (!tracker->getMapPoints().empty()) {
                    ros_publisher->publishPointCloud(tracker->getMapPoints());
                    ros_publisher->publishCameraPose(tracker->getCurrentPose());
                    
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(ros_publisher->get_logger(), "Error: %s", e.what());
            }
        }
    });
    
    rclcpp::spin(ros_publisher);
    
    running = false;
    slam_thread.join();
    delete tracker;
    rclcpp::shutdown();
    delete(tracker);
    return 0;
}