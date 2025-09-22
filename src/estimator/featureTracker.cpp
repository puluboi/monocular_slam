#include "featureTracker.hpp"
featureTracker::featureTracker(){
    orb = cv::ORB::create(500);
    if(!cam.initialize()) {
       throw std::runtime_error("Camera failed to initialize!");
       return;
    }
    if(!cam.startCapture()) {
        throw std::runtime_error("Camera failed to start capture!");
        return;
    }
}

void featureTracker::orbTest()
{
    cv::Mat image = cam.captureFrame();
    if (orb == nullptr) {
        throw std::runtime_error("ORB feature detector is not initialized.");
        return;
    }
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    
    cv::Mat output;
    cv::drawKeypoints(image, keypoints, output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("ORB Keypoints", output);
    cv::waitKey(1);  // Changed from waitKey(0) to waitKey(1) for real-time updates
}
