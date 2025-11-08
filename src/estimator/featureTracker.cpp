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

    has_previous_frame = false;
    last_frame_time = std::chrono::steady_clock::now();
    current_pose = cv::Mat::eye(4, 4, CV_64F);
    previous_pose = current_pose.clone();
    world_pose_ = current_pose.clone();
    
    // Start IMU update thread
    float ax_off, ay_off, az_off;
    float gr_off, gp_off, gy_off;
    
    //std::cout << "Keep the sensor COMPLETELY STILL on a flat surface.\n";
    //std::cout << "Calculating offsets (this takes ~10 seconds)...\n";
    //
    //imu.getOffsets(&ax_off, &ay_off, &az_off, &gr_off, &gp_off, &gy_off);
    //
    //std::cout << "\n// Accelerometer offsets:\n";
    //std::cout << "#define A_OFF_X " << (int)ax_off << "\n";
    //std::cout << "#define A_OFF_Y " << (int)ay_off << "\n";
    //std::cout << "#define A_OFF_Z " << (int)az_off << "\n";
    //
    //std::cout << "\n// Gyroscope offsets:\n";
    //std::cout << "#define G_OFF_X " << (int)gr_off << "\n";
    //std::cout << "#define G_OFF_Y " << (int)gp_off << "\n";
    //std::cout << "#define G_OFF_Z " << (int)gy_off << "\n";
}//

void featureTracker::orbTest()
{
    // Integrate IMU data at current time
    auto current_time = std::chrono::steady_clock::now();
    integrateIMU(current_time);
    
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
        F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 10.0, 0.8, inliers);
        if (F.empty() || F.rows != 3 || F.cols != 3 || inliers.empty()) {
            //std::cout << "Fundamental matrix computation failed - skipping frame" << std::endl;
            renderImg(tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
       
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
            //std::cout << "Insufficient inliers (" << pts1_inliers.size() 
            //          << ") - skipping frame" << std::endl;
            renderImg(tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            return;
        }

        // Decompose the Essential matrix to get R and t
        cv::Mat R, t, mask;
        int inliers_pose = cv::recoverPose(E, pts1_inliers, pts2_inliers, K, R, t, mask);
        if (R.empty() || t.empty() || inliers_pose < 8) {
            //std::cout << "Pose recovery failed - skipping frame" << std::endl;
            renderImg(tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            return;
        }
        std::cout<< "s";
        cv::Mat pose1 = world_pose_.clone();
        
        cv::Mat delta = cv::Mat::eye(4, 4, CV_64F); 
        R.copyTo(delta(cv::Rect(0, 0, 3, 3)));
        t.copyTo(delta(cv::Rect(3, 0, 1, 3)));
        
        // DEBUG: Print to verify structure
        std::cout << "Delta transformation matrix:\n" << delta << std::endl;
        std::cout << "t vector:\n" << t << std::endl;

        fuseVisualIMU(&delta);

        // Apply visual odometry delta transform
        world_pose_ = world_pose_ * delta;
        
        // Then fuse with IMU estimate (blends the result)
        //fuseVisualIMU(R, t);
        
        // Update pose tracking
        previous_pose = current_pose.clone();
        current_pose = world_pose_.clone();
        
        // Triangulate 3D points from 2D tracking
        std::vector<cv::Point3f> points_3d = triangulatePoints(
            pose1, current_pose, pts1_inliers, pts2_inliers, K);
        if(!points_3d.empty()){
            map_points_3d.insert(map_points_3d.end(), points_3d.begin(), points_3d.end());
         
        }
        renderImg(tracking_viz);
        
        }else {
        // First frame - just show keypoints
        cv::drawKeypoints(image, curr_keypoints, output, cv::Scalar::all(-1), 
                         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //cv::imshow("ORB Keypoints", output);
        
        std::cout << ".";
        }
    
    // Store current frame data for next iteration
    prev_image = image.clone();
    prev_keypoints = curr_keypoints;
    prev_descriptors = curr_descriptors.clone();
    has_previous_frame = true;
    
    // Update time for next IMU integration
    last_frame_time = current_time;
}

void featureTracker::reset(){
    map_points_3d.clear();
    has_previous_frame = false;
    prev_keypoints.clear();
    prev_descriptors.release();
    curr_keypoints.clear();
    curr_descriptors.release();
    matches.clear();
    
    current_pose = cv::Mat::eye(4, 4, CV_64F);
    previous_pose = current_pose.clone();
    world_pose_ = current_pose.clone();
    
    orientation_euler = cv::Vec3d(0.0, 0.0, 0.0);
    velocity = cv::Vec3d(0.0, 0.0, 0.0);
    
    last_frame_time = std::chrono::steady_clock::now();
}
std::vector<cv::Point3f> featureTracker::triangulatePoints(
        const cv::Mat& pose1, const cv::Mat& pose2,
        const std::vector<cv::Point2f>& pts1,
        const std::vector<cv::Point2f>& pts2,
        const cv::Mat& K)
{
    std::vector<cv::Point3f> points_3d;
    
    // Invert poses to get camera-to-world transformations
    // (pose matrices are world-to-camera, we need camera-to-world for projection)
    cv::Mat pose1_inv, pose2_inv;
    cv::invert(pose1, pose1_inv);
    cv::invert(pose2, pose2_inv);
    
    // Build projection matrices P = K * [R|t]
    cv::Mat P1 = K * pose1_inv.rowRange(0, 3);
    cv::Mat P2 = K * pose2_inv.rowRange(0, 3);

    for (size_t i = 0; i < pts1.size(); ++i) {
        cv::Mat A(4, 4, CV_64F);
        double u1 = pts1[i].x, v1 = pts1[i].y;
        double u2 = pts2[i].x, v2 = pts2[i].y;

        // Build linear system: A * X = 0
        // Following DLT (Direct Linear Transform) triangulation
        A.row(0) = u1 * P1.row(2) - P1.row(0);
        A.row(1) = v1 * P1.row(2) - P1.row(1);
        A.row(2) = u2 * P2.row(2) - P2.row(0);
        A.row(3) = v2 * P2.row(2) - P2.row(1);

        cv::SVD svd(A, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        cv::Mat X = svd.vt.row(3).t(); // homogeneous 4x1 (last row of V^T)

        double w = X.at<double>(3, 0);
        if (std::abs(w) < 1e-9) continue;  // Skip invalid points

        // Convert from homogeneous to 3D Euclidean coordinates
        cv::Point3f pt(
            static_cast<float>(X.at<double>(0, 0) / w),
            static_cast<float>(X.at<double>(1, 0) / w),
            static_cast<float>(X.at<double>(2, 0) / w));
        
        
        points_3d.push_back(pt);
    }
    return points_3d;
}

void featureTracker::renderImg(cv::Mat img){
    if(can_showImg){
        cv::imshow("Motion Vectors", img);
        int key = cv::waitKey(1);
        if (key == 'r' || key == 'R') {
            std::cout << "Resetting tracker (keyboard 'r')." << std::endl;
            reset();
            if (ros_publisher_) {
                ros_publisher_->publishCameraPose(current_pose);
            }
        }
    }
}

void featureTracker::integrateIMU(const std::chrono::steady_clock::time_point& current_time) {
    
    auto dt = std::chrono::duration<double>(current_time - last_frame_time).count();
    if (dt > 1.0 || dt <= 0) return; // Skip if dt is unreasonable

    
    // IMU physical orientation: X=up, Y=left, Z=forward
    // OpenCV convention: X=right, Y=down, Z=forward
    // Transformation: CV_X = -IMU_Y, CV_Y = -IMU_X, CV_Z = IMU_Z
    
    float imu_roll, imu_pitch, imu_yaw;
    imu.getAngle(0, &imu_roll);   // IMU X-axis (up)
    imu.getAngle(1, &imu_pitch);  // IMU Y-axis (left)
    imu.getAngle(2, &imu_yaw);    // IMU Z-axis (forward)
    
    // Convert degrees to radians (MPU6050 returns angles in degrees)
    const double DEG_TO_RAD = M_PI / 180.0;
    
    // Remap orientation to OpenCV frame
    // Roll around X(right) = -pitch around Y(left)
    // Pitch around Y(down) = -roll around X(up)
    // Yaw around Z(forward) = yaw around Z(forward)
    orientation_euler[0] = static_cast<double>(-imu_pitch * DEG_TO_RAD);  // Roll (around X/right)
    orientation_euler[1] = static_cast<double>(-imu_roll * DEG_TO_RAD);   // Pitch (around Y/down)
    orientation_euler[2] = static_cast<double>(imu_yaw * DEG_TO_RAD);     // Yaw (around Z/forward)
    
    std::cout << "IMU(deg) x:" << orientation_euler[0] << " y:" << orientation_euler[1] << " z:" << orientation_euler[2] << std::endl; 

    float imu_acx, imu_acy, imu_acz;
    imu.getAccel(&imu_acx, &imu_acy, &imu_acz);
    
    // Remap accelerometer: OpenCV(right, down, forward) <- IMU(up, left, forward)
    float acx = -imu_acy;  // OpenCV X (right) = -IMU Y (left)
    float acy = -imu_acx;  // OpenCV Y (down) = -IMU X (up)
    float acz = imu_acz;   // OpenCV Z (forward) = IMU Z (forward)
    
    // Get rotation matrix from current orientation
    cv::Mat R = eulerToRotationMatrix(orientation_euler);
    
    // Transform acceleration to world frame (gravity already compensated by MPU6050 library)
    cv::Mat acc_body = (cv::Mat_<double>(3, 1) << acx, acy, acz);
    cv::Mat acc_world = R * acc_body;
    
    // Update velocity (v = v0 + a*dt)
    velocity[0] += acc_world.at<double>(0) * dt;
    velocity[1] += acc_world.at<double>(1) * dt;
    velocity[2] += acc_world.at<double>(2) * dt;
  
  
  
}

cv::Mat featureTracker::eulerToRotationMatrix(const cv::Vec3d& euler) {
    double roll = euler[0], pitch = euler[1], yaw = euler[2];
    
    // Rotation matrices for each axis
    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll));
    
    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch));
    
    cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);
    
    return Rz * Ry * Rx;
}

void featureTracker::fuseVisualIMU(cv::Mat *delta) {
    // Extract rotation matrix from visual odometry delta
    cv::Mat visual_R = (*delta)(cv::Rect(0, 0, 3, 3)).clone();
    
    // Convert visual rotation matrix to Euler angles
    cv::Vec3d visual_euler;
    
    // Extract Euler angles from rotation matrix (ZYX convention)
    // Based on: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    visual_euler[1] = asin(-visual_R.at<double>(2, 0));  // pitch
    
    if (cos(visual_euler[1]) > 1e-6) {
        visual_euler[0] = atan2(visual_R.at<double>(2, 1), visual_R.at<double>(2, 2));  // roll
        visual_euler[2] = atan2(visual_R.at<double>(1, 0), visual_R.at<double>(0, 0));  // yaw
    } else {
        // Gimbal lock case
        visual_euler[0] = 0;
        visual_euler[2] = atan2(-visual_R.at<double>(0, 1), visual_R.at<double>(1, 1));
    }
    
    // Hybrid fusion:
    // - Roll & Pitch: Use IMU (more stable for fast rotations)
    // - Yaw: Use Visual (IMU yaw drifts without magnetometer)
    cv::Vec3d fused_euler;
    fused_euler[0] = orientation_euler[0];  // Roll from IMU
    fused_euler[1] = orientation_euler[1];  // Pitch from IMU
    fused_euler[2] = orientation_euler[2];       // Yaw from IMU
    
    // Convert fused Euler angles back to rotation matrix
    cv::Mat fused_R = eulerToRotationMatrix(fused_euler);
    
    // Update delta with fused rotation (keep visual translation)
    fused_R.copyTo((world_pose_)(cv::Rect(0, 0, 3, 3)));
    
    cv::Mat visual_t = (*delta)(cv::Rect(3, 0, 1, 3)).clone();
    auto current_time = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration<double>(current_time - last_frame_time).count();
    
    if (dt > 0.001 && dt < 1.0) {  // Reasonable dt
        // IMU displacement over dt: d = v * dt
        cv::Mat imu_displacement = (cv::Mat_<double>(3, 1) << 
            velocity[0] * dt,
            velocity[1] * dt,
            velocity[2] * dt);
        
        // Calculate scale from IMU
        double imu_dist = cv::norm(imu_displacement);
        double visual_dist = cv::norm(visual_t);
        
        double scale = 1.0;  // Default to visual-only
        
        if (visual_dist > 1e-6 && imu_dist > 1e-3) {  // Both have meaningful motion
            // Use IMU to scale visual translation
            scale = imu_dist / visual_dist;
            
            // Clamp scale to prevent extreme values
            scale = std::max(0.1, std::min(scale, 10.0));
        }
        
        // Scale visual translation
        cv::Mat scaled_visual_t = visual_t * scale;
        
        // Blend: 70% scaled visual + 30% IMU displacement
        // (Visual is more reliable for direction, IMU provides scale)
        double alpha = 0.7;  // Weight for visual
        cv::Mat fused_t = alpha * scaled_visual_t + (1.0 - alpha) * imu_displacement;
        
        // Update delta with fused translation
        fused_t.copyTo((*delta)(cv::Rect(3, 0, 1, 3)));
        
        std::cout << "Scale: " << scale 
                  << ", Visual dist: " << visual_dist 
                  << ", IMU dist: " << imu_dist << std::endl;
    } else {
        // Invalid dt, use visual translation only
        std::cout << "Invalid dt, using visual-only translation" << std::endl;
    }
    
}

