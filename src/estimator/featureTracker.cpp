#include "featureTracker.hpp"
#include "../ros2/PointCloudPublisher.hpp"
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
            if (matches[i].distance < 75) { // match sensitivity: lower values for higher accuracy
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
        
        //std::cout << "Features: " << curr_keypoints.size() 
          //        << ", Good Matches: " << good_matches.size() << std::endl;
        
        // Draw tracking trails on current frame
        cv::Mat tracking_viz = image.clone();
        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(good_matches.size());
        pts2.reserve(good_matches.size());
        if(good_matches.size()<=2){
            // Update frame buffers before returning
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            return;
        }
        for (const auto& match : good_matches) {
            cv::Point2f prev_pt = prev_keypoints[match.queryIdx].pt;
            cv::Point2f curr_pt = curr_keypoints[match.trainIdx].pt;

            pts1.push_back(prev_pt);
            pts2.push_back(curr_pt);

            // Draw arrow showing motion
            cv::arrowedLine(tracking_viz, prev_pt, curr_pt, 
                           cv::Scalar(0, 255, 255), 2, 8, 0, 0.3);
            cv::circle(tracking_viz, curr_pt, 3, cv::Scalar(0, 0, 255), -1);
        }
        cv::Mat F, inliers;
        // Pi Camera 3 Wide intrinsic parameters
        double sensor_width_mm = 6.45;
        double focal_length_mm = 4.74;

        double fx = (focal_length_mm * cam.getResolution().first)/sensor_width_mm;  // Focal length in x (pixels)
        double fy = fx;  // Focal length in y (pixels)  
        double cx = cam.getResolution().first/2;  // Principal point x (image center)
        double cy = cam.getResolution().second/2;;  // Principal point y (image center)
        
        cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                     0, fy, cy,
                     0, 0, 1);
        F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 10.0, 0.99, inliers);
        if (F.empty() || F.rows != 3 || F.cols != 3 || inliers.empty()) {
            std::cout << "Fundamental matrix computation failed - skipping frame" << std::endl;
            cv::imshow("Motion Vectors", tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            cv::waitKey(1);
            return;
        }
        cv::Mat E = K.t() * F * K;
        
        // Extract inlier points for pose recovery
        
        std::vector<cv::Point2f> pts1_inliers, pts2_inliers;
        for (int i = 0; i < inliers.rows; i++) {
            if (inliers.at<uchar>(i)) {
                pts1_inliers.push_back(pts1[i]);
                pts2_inliers.push_back(pts2[i]);
            }
        }

        //  Check for minimum points before pose recovery
        if (pts1_inliers.size() < 8) {
            std::cout << "Insufficient inliers (" << pts1_inliers.size() 
                      << ") - skipping frame" << std::endl;
            cv::imshow("Motion Vectors", tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            cv::waitKey(1);
            return;
        }

        // Decompose the Essential matrix to get R and t
        cv::Mat R, t, mask;
        int inliers_pose = cv::recoverPose(E, pts1_inliers, pts2_inliers, K, R, t, mask);
        if (R.empty() || t.empty() || inliers_pose < 8) {
            std::cout << "Pose recovery failed - skipping frame" << std::endl;
            cv::imshow("Motion Vectors", tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            cv::waitKey(1);
            return;
        }
        std::cout<< "pose succcess!"<<std::endl;
        cv::Mat pose1 = cv::Mat::eye(4, 4, CV_64F);  // Previous frame at origin
        
        cv::Mat pose2 = cv::Mat::eye(4, 4, CV_64F);  // Current frame
        R.copyTo(pose2(cv::Rect(0, 0, 3, 3)));       // Copy rotation
        t.copyTo(pose2(cv::Rect(3, 0, 1, 3)));       // Copy translation
        
        // Triangulate 3D points from 2D tracking
        std::vector<cv::Point3f> points_3d = triangulatePoints(
            pose1, pose2, pts1_inliers, pts2_inliers, K);
        if(!points_3d.empty()){
            map_points_3d.insert(map_points_3d.end(), points_3d.begin(), points_3d.end());
         
        }
        
        
        cv::imshow("Motion Vectors", tracking_viz);
        
    } else {
        // First frame - just show keypoints
        cv::drawKeypoints(image, curr_keypoints, output, cv::Scalar::all(-1), 
                         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //cv::imshow("ORB Keypoints", output);
        
        std::cout << "First frame - Features detected: " << curr_keypoints.size() << std::endl;
    }
    
    // Store current frame data for next iteration
    prev_image = image.clone();
    prev_keypoints = curr_keypoints;
    prev_descriptors = curr_descriptors.clone();
    has_previous_frame = true;
    
    cv::waitKey(1);
}

std::vector<cv::Point3f> featureTracker::triangulatePoints(const cv::Mat &pose1, const cv::Mat &pose2, const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, const cv::Mat &K)
{
    std::vector<cv::Point3f> points_3d;
    
    cv::Mat P1 = K* pose1.rowRange(0,3);
    cv::Mat P2 = K* pose2.rowRange(0,3);

    cv::Mat pts1_homog, pts2_homog;

    cv::convertPointsToHomogeneous(cv::Mat(pts1).reshape(1), pts1_homog);
    cv::convertPointsToHomogeneous(cv::Mat(pts2).reshape(1), pts2_homog);

    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1,pts2, points4D);

    // Convert from homogeneous to 3D coordinates
    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat point = points4D.col(i);
        // Convert from homogeneous [X, Y, Z, W] to 3D [X/W, Y/W, Z/W]
        float w = point.at<float>(3, 0);
        if (std::abs(w) > 1e-6) {  // Avoid division by zero
            cv::Point3f pt3d(
                point.at<float>(0, 0) / w,
                point.at<float>(1, 0) / w,
                point.at<float>(2, 0) / w
            );
            
            // Filter out points that are too far or behind camera
            if (pt3d.z > 0 && pt3d.z < 100.0) {  // Between 0 and 100 meters
                std::cout<< "Point: "<< pt3d.x<<" "<< pt3d.y<< " "<< pt3d.z<< std::endl;
                points_3d.push_back(pt3d);
            }
        }
    }
    //std::cout << "Pointcloud size"<< points_3d.size()<< "------------------------!" << std::endl;
    return points_3d;
}

