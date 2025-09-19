#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera/camera.hpp"

int main(int, char**) {
    std::cout << "Monocular SLAM - Camera Stream" << std::endl;
    
    // Initialize camera
    PiCamera camera;
    
    if (!camera.initialize()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return -1;
    }
    
    if (!camera.startCapture()) {
        std::cerr << "Failed to start camera capture" << std::endl;
        return -1;
    }
    
    // Create display window
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    
    std::cout << "Streaming started. Press 'q' or ESC to quit." << std::endl;
    
    // Main capture loop
    while (true) {
        cv::Mat frame = camera.captureFrame();
        
        if (!frame.empty()) {
            cv::imshow("Camera Feed", frame);
        }
        
        // Check for quit
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            break;
        }
    }
    
    // Cleanup
    cv::destroyAllWindows();
    camera.stopCapture();
    camera.release();
    
    std::cout << "Application ended." << std::endl;
    return 0;
}
