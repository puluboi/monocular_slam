#include "featureTracker.hpp"

/// @brief Check if the input frame is too blurry based on Laplacian variance
/// @param processed_image The input image to check for blur
/// @param blur_threshold The threshold value below which the frame is considered blurry
/// @return true if the frame is blurry, false otherwise
bool featureTracker::isFrameBlurry(const cv::Mat& processed_image, double blur_threshold)
    {
        cv::Mat laplacian;
        cv::Laplacian(processed_image, laplacian, CV_64F);
        cv::Scalar mu, sigma;
        cv::meanStdDev(laplacian, mu, sigma);
        double blur_score = sigma.val[0] * sigma.val[0];
        
        if (blur_score < blur_threshold)
        {
            return true;
        }
        return false;
    }