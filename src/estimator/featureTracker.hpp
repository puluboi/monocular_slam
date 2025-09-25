#pragma once

#include <opencv2/opencv.hpp>
#include "camera.hpp"

class featureTracker{
public:
    featureTracker();
    void orbTest();

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
};