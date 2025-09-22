#pragma once

#include <opencv2/opencv.hpp>
#include "camera.hpp"

class featureTracker{
public:
    featureTracker();
    void orbTest();

private:
    PiCamera cam;
    cv::Ptr<cv::ORB> orb;
    cv::Mat curr_descriptors;
    cv::Mat prev_descriptors;

    std::vector<cv::KeyPoint> curr_keypoints;
    std::vector<cv::KeyPoint> prev_keypoints;
};