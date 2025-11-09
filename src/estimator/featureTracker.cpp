#include "featureTracker.hpp"

#include "../ros2/PointCloudPublisher.hpp"
featureTracker::featureTracker()
{
    orb = cv::ORB::create(
        2000,           // nfeatures - more features for robustness
        1.2f,           // scaleFactor - better scale invariance
        8,              // nlevels - more pyramid levels
        31,             // edgeThreshold - balance detection on edges
        0,              // firstLevel
        2,              // WTA_K - use 2 for better distinctiveness
        cv::ORB::HARRIS_SCORE,  // Use Harris corner score (better than FAST)
        31,             // patchSize
        20              // fastThreshold - lower for more features in low-texture
    );
    
    bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    if (!cam.initialize())
    {
        throw std::runtime_error("Camera failed to initialize!");
        return;
    }
    if (!cam.startCapture())
    {
        throw std::runtime_error("Camera failed to start capture!");
        return;
    }


    has_previous_frame = false;
    last_frame_time = std::chrono::steady_clock::now();
    current_pose = cv::Mat::eye(4, 4, CV_64F);
    previous_pose = current_pose.clone();
    world_pose_ = current_pose.clone();

    
    
   
    // std::cout << "Keep the sensor COMPLETELY STILL on a flat surface.\n";
    // std::cout << "Calculating offsets (this takes ~10 seconds)...\n";
    //
    // imu.getOffsets(&ax_off, &ay_off, &az_off, &gr_off, &gp_off, &gy_off);
    //
    // std::cout << "\n// Accelerometer offsets:\n";
    // std::cout << "#define A_OFF_X " << (int)ax_off << "\n";
    // std::cout << "#define A_OFF_Y " << (int)ay_off << "\n";
    // std::cout << "#define A_OFF_Z " << (int)az_off << "\n";
    //
    // std::cout << "\n// Gyroscope offsets:\n";
    // std::cout << "#define G_OFF_X " << (int)gr_off << "\n";
    // std::cout << "#define G_OFF_Y " << (int)gp_off << "\n";
    // std::cout << "#define G_OFF_Z " << (int)gy_off << "\n";
} //

void featureTracker::orbTest()
{
    // Integrate IMU data at current time
    auto current_time = std::chrono::steady_clock::now();
    // integrateIMU(current_time);
    auto [roll, pitch, yaw] = estimator.getOrientation();
    std::cout << "Orientation: " << roll << ", " << pitch << ", " << yaw << std::endl;

    cv::Mat image = cam.captureFrame();
    if(isFrameBlurry(image, 80)){
        std::cout<< "b";
    }
    if (orb == nullptr)
    {
        throw std::runtime_error("ORB feature detector is not initialized.");
        return;
    }

    orb->detectAndCompute(image, cv::noArray(), curr_keypoints, curr_descriptors);

    cv::Mat output;

    // Track keypoints between frames
    if (has_previous_frame && !prev_descriptors.empty() && !curr_descriptors.empty())
    {
        bf->match(prev_descriptors, curr_descriptors, matches);

        // Sort matches by distance (best matches first)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Filter good matches
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < matches.size() && i < 100; i++)
        {
            if (matches[i].distance < 72)
            { // match sensitivity: lower values for higher accuracy
                good_matches.push_back(matches[i]);
            }
        }

        // Draw matches between previous and current frame
        cv::drawMatches(prev_image, prev_keypoints, image, curr_keypoints,
                        good_matches, output,
                        cv::Scalar(0, 255, 0), // Green lines for matches
                        cv::Scalar(255, 0, 0), // Blue circles for keypoints
                        std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // cv::imshow("Feature Tracking", output); // Compare features.

        // std::cout << "Features: " << curr_keypoints.size()
        //         << ", Good Matches: " << good_matches.size() << std::endl;

        // Draw tracking trails on current frame
        cv::Mat tracking_viz = image.clone();
        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(good_matches.size());
        pts2.reserve(good_matches.size());
        if (good_matches.size() <= 2)
        {
            // Update frame buffers before returning
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            return;
        }
        for (const auto &match : good_matches)
        {
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
        
        double fx = (focal_length_mm * cam.getResolution().first) / sensor_width_mm; // Focal length in x (pixels)
        double fy = fx;                                                              // Focal length in y (pixels)
        double cx = cam.getResolution().first / 2;                                   // Principal point x (image center)
        double cy = cam.getResolution().second / 2;
        ; // Principal point y (image center)

        cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                     0, fy, cy,
                     0, 0, 1);
        F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 10.0, 0.999, inliers);
        if (F.empty() || F.rows != 3 || F.cols != 3 || inliers.empty())
        {
            // std::cout << "Fundamental matrix computation failed - skipping frame" << std::endl;
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
        for (int i = 0; i < inliers.rows; i++)
        {
            if (inliers.at<uchar>(i))
            {
                pts1_inliers.push_back(pts1[i]);
                pts2_inliers.push_back(pts2[i]);
            }
        }

        //  Check for minimum points before pose recovery
        if (pts1_inliers.size() < 8)
        {
            // std::cout << "Insufficient inliers (" << pts1_inliers.size()
            //           << ") - skipping frame" << std::endl;
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
        if (R.empty() || t.empty() || inliers_pose < 8)
        {
            // std::cout << "Pose recovery failed - skipping frame" << std::endl;
            renderImg(tracking_viz);
            prev_image = image.clone();
            prev_keypoints = curr_keypoints;
            prev_descriptors = curr_descriptors.clone();
            has_previous_frame = true;
            return;
        }
        std::cout << "s";
        cv::Mat pose1 = world_pose_.clone();

        cv::Mat delta = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(delta(cv::Rect(0, 0, 3, 3))); 
        t.copyTo(delta(cv::Rect(3, 0, 1, 3))); // magnitude of t is one unit, ie recover pose only gives the direction of movement.
        // Print camera movement direction based on translation vector t
        //double tx = t.at<double>(0);
        //double ty = t.at<double>(1);
        //double tz = t.at<double>(2);

        //std::cout << "Camera Movement: ";
        //if (std::abs(tx) < 0.01 && std::abs(ty) < 0.01 && std::abs(tz) < 0.01)
        //{
        //    std::cout << "Stationary";
        //}
        //else
        //{
        //    double abs_tx = std::abs(tx);
        //    double abs_ty = std::abs(ty);
        //    double abs_tz = std::abs(tz);
        //    
        //    if (abs_tx >= abs_ty && abs_tx >= abs_tz)
        //    {
        //    std::cout << (tx > 0 ? "Right" : "Left");
        //    }
        //    else if (abs_ty >= abs_tx && abs_ty >= abs_tz)
        //    {
        //    std::cout << (ty > 0 ? "Down" : "Up");
        //    }
        //    else
        //    {
        //    std::cout << (tz > 0 ? "Forward" : "Backward");
        //    }
        //}
        //std::cout << " (" << tx << ", " << ty << ", " << tz << ")" << std::endl;

        // Print rotation direction from delta rotation matrix
        //cv::Mat delta_R = delta(cv::Rect(0, 0, 3, 3));
        //double roll = atan2(-delta_R.at<double>(1, 0), delta_R.at<double>(2, 2));
        //double pitch = asin(-delta_R.at<double>(2, 1));
        //double yaw = atan2(-delta_R.at<double>(2
        //    , 0), delta_R.at<double>(0, 0));
//
        //double abs_roll = std::abs(roll);
        //double abs_pitch = std::abs(pitch);
        //double abs_yaw = std::abs(yaw);
        //
        //std::cout << "Rotation: ";
        //if (abs_roll >= abs_pitch && abs_roll >= abs_yaw)
        //{
        //    std::cout << (roll > 0 ? "Roll Right" : "Roll Left");
        //}
        //else if (abs_pitch >= abs_roll && abs_pitch >= abs_yaw)
        //{
        //    std::cout << (pitch > 0 ? "Pitch Up" : "Pitch Down");
        //}
        //else
        //{
        //    std::cout << (yaw > 0 ? "Yaw Left" : "Yaw Right");
        //}
        //std::cout << " (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
        world_pose_ = world_pose_ * delta;

        // Then fuse with IMU estimate (blends the result)
        // fuseVisualIMU(R, t);

        // Update pose tracking
        previous_pose = current_pose.clone();
        current_pose = world_pose_.clone();

        // Triangulate 3D points from 2D tracking
        std::vector<cv::Point3f> raw_pts = triangulatePoints(
            pose1, current_pose, pts1_inliers, pts2_inliers, K);
        if (!raw_pts.empty())
        {
            for (size_t i = 0; i < raw_pts.size(); ++i)
            {
                const cv::Point3f &Pw = raw_pts[i];
                // Reproject to camera 1
                cv::Mat Pw_h = (cv::Mat_<double>(4, 1) << Pw.x, Pw.y, Pw.z, 1.0);
                cv::Mat Pc1 = pose1.rowRange(0, 3) * Pw_h;
                cv::Mat Pc2 = current_pose.rowRange(0, 3) * Pw_h;

                double z1 = Pc1.at<double>(2);
                double z2 = Pc2.at<double>(2);
                if (z1 <= 0 || z2 <= 0)
                    continue; // enforce positive depth

                // Parallax check (angle between viewing rays)
                cv::Point3d ray1 = cv::Point3d(Pc1) / z1;
                cv::Point3d ray2 = cv::Point3d(Pc2) / z2;
                double cos_angle = ray1.dot(ray2) /
                                   (cv::norm(ray1) * cv::norm(ray2));
                double parallax = std::acos(std::min(1.0, std::max(-1.0, cos_angle)));
                if (parallax < M_1_PI*1.f/180.f)
                    continue; // reject tiny parallax

                map_points_3d.push_back(Pw);
            }
        }
        renderImg(tracking_viz);
    }
    else
    {
        // First frame - just show keypoints
        cv::drawKeypoints(image, curr_keypoints, output, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::imshow("ORB Keypoints", output);

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

void featureTracker::reset()
{
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
    const cv::Mat &pose1, const cv::Mat &pose2,
    const std::vector<cv::Point2f> &pts1,
    const std::vector<cv::Point2f> &pts2,
    const cv::Mat &K)
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

    for (size_t i = 0; i < pts1.size(); ++i)
    {
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
        if (std::abs(w) < 1e-9)
            continue; // Skip invalid points

        // Convert from homogeneous to 3D Euclidean coordinates
        cv::Point3f pt(
            static_cast<float>(X.at<double>(0, 0) / w),
            static_cast<float>(X.at<double>(1, 0) / w),
            static_cast<float>(X.at<double>(2, 0) / w));

        points_3d.push_back(pt);
    }
    return points_3d;
}

void featureTracker::renderImg(cv::Mat img)
{
    if (can_showImg)
    {
        cv::imshow("Motion Vectors", img);
        int key = cv::waitKey(1);
        if (key == 'r' || key == 'R')
        {
            std::cout << "Resetting tracker (keyboard 'r')." << std::endl;
            reset();
            if (ros_publisher_)
            {
                ros_publisher_->publishCameraPose(current_pose);
            }
        }
    }
}



cv::Mat featureTracker::eulerToRotationMatrix(const cv::Vec3d &euler)
{
    double roll = euler[0], pitch = euler[1], yaw = euler[2];

    // Rotation matrices for each axis
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
                  0, 1, 0,
                  -sin(pitch), 0, cos(pitch));

    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1);

    return Rz * Ry * Rx;
}

