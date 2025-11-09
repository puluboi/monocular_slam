#pragma once

#include <opencv2/opencv.hpp>
#include "../camera/camera.hpp"
#include "imuEstimator.hpp"
#include <memory>
#include <chrono>

class PointCloudPublisher;

class featureTracker{
public:
    featureTracker();
    void orbTest();
    void setROSPublisher(std::shared_ptr<PointCloudPublisher> publisher) {
        ros_publisher_ = publisher;
    }
    void reset();
    std::vector<cv::Point3f> triangulatePoints(
        const cv::Mat &pose1,
        const cv::Mat &pose2,
        const std::vector<cv::Point2f> &pts1,
        const std::vector<cv::Point2f> &pts2,
        const cv::Mat &K);

    void renderImg(cv::Mat img);

    void integrateIMU(const std::chrono::steady_clock::time_point &current_time);

    cv::Mat eulerToRotationMatrix(const cv::Vec3d &euler);

    void fuseVisualIMU(cv::Mat *delta);

    // Getter methods for ROS 2 integration
    std::vector<cv::Point3f> getMapPoints() const { 
        return map_points_3d; 
    }
    
    cv::Mat getCurrentPose() const {
        if (!current_pose.empty()) {
            return current_pose.clone();
        }
        return world_pose_.clone();
    }
    
    cv::Mat getTrackingVisualization() {
        // Return the last tracking visualization image
        if (!prev_image.empty()) {
            return prev_image.clone();
        }
        return cv::Mat();
    }
private:
    // Integrated pose from IMU
    cv::Vec3d velocity = cv::Vec3d(0, 0, 0);
    cv::Vec3d orientation_euler = cv::Vec3d(0, 0, 0);  // roll, pitch, yaw in radians 
     // Time tracking for IMU integration
    std::chrono::steady_clock::time_point last_frame_time;
    bool can_showImg = true;
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
    imuEstimator estimator;
    std::vector<cv::Point3f> map_points_3d;
    cv::Mat current_pose;
    cv::Mat previous_pose;
    cv::Mat world_pose_ = cv::Mat::eye(4, 4, CV_64F);
    std::shared_ptr<PointCloudPublisher> ros_publisher_;
    bool isFrameBlurry(const cv::Mat &processed_image, double blur_threshold);
};