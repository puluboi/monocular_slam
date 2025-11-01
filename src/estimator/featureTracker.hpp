#pragma once

#include <opencv2/opencv.hpp>
#include "../camera/camera.hpp"

#include <memory>
class PointCloudPublisher;

class featureTracker{
public:
    featureTracker();
    void orbTest();
    void setROSPublisher(std::shared_ptr<PointCloudPublisher> publisher) {
        ros_publisher_ = publisher;
    }
    std::vector<cv::Point3f> triangulatePoints(
        const cv::Mat& pose1, 
        const cv::Mat& pose2,
        const std::vector<cv::Point2f>& pts1,
        const std::vector<cv::Point2f>& pts2,
        const cv::Mat& K);



    // Getter methods for ROS 2 integration
    std::vector<cv::Point3f> getMapPoints() const { 
        return map_points_3d; 
    }
    
    cv::Mat getCurrentPose() const { 
        return current_pose.clone(); 
    }
    
    cv::Mat getTrackingVisualization() {
        // Return the last tracking visualization image
        if (!prev_image.empty()) {
            return prev_image.clone();
        }
        return cv::Mat();
    }
private:
    PiCamera cam;
    bool has_previous_frame;
    cv::Mat prev_image;
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> bf;
    cv::Mat curr_descriptors;
    cv::Mat prev_descriptors;
    std::vector<cv::DMatch> matches;

    std::vector<cv::KeyPoint> curr_keypoints;
    std::vector<cv::KeyPoint> prev_keypoints;

    std::vector<cv::Point3f> map_points_3d;
    cv::Mat current_pose;
    cv::Mat previous_pose;
    
    std::shared_ptr<PointCloudPublisher> ros_publisher_;
};