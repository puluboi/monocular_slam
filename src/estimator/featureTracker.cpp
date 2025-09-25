#include "featureTracker.hpp"
featureTracker::featureTracker(){
    orb = cv::ORB::create(1500);
    bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
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
    
    orb->detectAndCompute(image, cv::noArray(), curr_keypoints, curr_descriptors);
    
    cv::Mat output;
    
    // Track keypoints between frames
    if (has_previous_frame && !prev_descriptors.empty() && !curr_descriptors.empty()) {
        bf->match(prev_descriptors, curr_descriptors, matches);
        
        // Sort matches by distance (best matches first)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });
        
        // Filter good matches
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < matches.size() && i < 100; i++) {
            if (matches[i].distance < 50) {
                good_matches.push_back(matches[i]);
            }
        }
        
        // Draw matches between previous and current frame
        cv::drawMatches(prev_image, prev_keypoints, image, curr_keypoints,
                       good_matches, output, 
                       cv::Scalar(0, 255, 0),      // Green lines for matches
                       cv::Scalar(255, 0, 0),      // Blue circles for keypoints
                       std::vector<char>(), 
                       cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        //cv::imshow("Feature Tracking", output); // Compare features.
        
        std::cout << "Features: " << curr_keypoints.size() 
                  << ", Good Matches: " << good_matches.size() << std::endl;
        
        // Optional: Draw tracking trails on current frame
        cv::Mat tracking_viz = image.clone();
        for (const auto& match : good_matches) {
            cv::Point2f prev_pt = prev_keypoints[match.queryIdx].pt;
            cv::Point2f curr_pt = curr_keypoints[match.trainIdx].pt;
            
            // Draw arrow showing motion
            cv::arrowedLine(tracking_viz, prev_pt, curr_pt, 
                           cv::Scalar(0, 255, 255), 2, 8, 0, 0.3);
            cv::circle(tracking_viz, curr_pt, 3, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("Motion Vectors", tracking_viz);
        
    } else {
        // First frame - just show keypoints
        cv::drawKeypoints(image, curr_keypoints, output, cv::Scalar::all(-1), 
                         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("ORB Keypoints", output);
        
        std::cout << "First frame - Features detected: " << curr_keypoints.size() << std::endl;
    }
    
    // Store current frame data for next iteration
    prev_image = image.clone();
    prev_keypoints = curr_keypoints;
    prev_descriptors = curr_descriptors.clone();
    has_previous_frame = true;
    
    cv::waitKey(1);
}