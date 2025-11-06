#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera/camera.hpp"

#include "estimator/featureTracker.hpp"

int main(int, char**) {
    std::cout << "Monocular SLAM - Feature Tracking" << std::endl;

    // Initialize feature tracker
    std::cout << "Initializing feature tracker..." << std::endl;
    featureTracker tracker;
    
    
    std::cout << "SLAM started. Press any key in the ORB window to continue, or Ctrl+C to quit." << std::endl;
    std::cout << "The 3D window shows your reconstructed point cloud!" << std::endl;
    std::cout << "Press ESC in the camera window to quit" << std::endl;
    
    // Main SLAM loop
    while (true) {
        // Run ORB feature detection and display
        tracker.orbTest();
        
        char key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
    }
    
    // Cleanup
    cv::destroyAllWindows();
    
    std::cout << "Application ended." << std::endl;
    return 0;
}
