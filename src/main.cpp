#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "camera/camera.hpp"
#include "IMU/IMU.hpp"
#include "estimator/featureTracker.hpp"

int main(int, char**) {
    std::cout << "Monocular SLAM - Feature Tracking" << std::endl;

    MPU6050 imu;
    
    // Initialize IMU (optional - comment out if not connected)
    std::cout << "Initializing IMU..." << std::endl;
    bool imu_init = imu.initialize();
    if (imu_init) {
        std::cout << "Calibrating IMU (keep still)..." << std::endl;
        imu.calibrate(100);  // Reduced samples for faster startup
        std::cout << "IMU calibration complete!" << std::endl;
    } else {
        std::cout << "IMU initialization failed - continuing without IMU" << std::endl;
    }
    
    // Initialize feature tracker
    std::cout << "Initializing feature tracker..." << std::endl;
    featureTracker tracker;
    
    std::cout << "SLAM started. Press any key in the ORB window to continue, or Ctrl+C to quit." << std::endl;
    
    int frame_count = 0;
    
    // Main SLAM loop
    while (true) {
        frame_count++;
        
        // Run ORB feature detection and display
        tracker.orbTest();
        
        // Read IMU data (less verbose output)
        if(imu_init) {
            MPU6050::IMUData imu_data;
            if (imu.readData(imu_data)) {
                // Only print IMU data every 10 frames to avoid spam
                if (frame_count % 10 == 0) {
                    float accel_x_g = imu_data.accel_x / 9.81f;
                    float accel_y_g = imu_data.accel_y / 9.81f;
                    float accel_z_g = imu_data.accel_z / 9.81f;
                    float total_g = sqrt(accel_x_g*accel_x_g + accel_y_g*accel_y_g + accel_z_g*accel_z_g);
                    
                    std::cout << "Frame " << frame_count << " | "
                              << "Accel: " << std::fixed << std::setprecision(2) << total_g << "g | "
                              << "Gyro: [" << std::setprecision(1)
                              << imu_data.gyro_x * 180.0f/M_PI << ", " 
                              << imu_data.gyro_y * 180.0f/M_PI << ", " 
                              << imu_data.gyro_z * 180.0f/M_PI << "]°/s"
                              << std::endl;
                }
            }
        }
    }
    
    // Cleanup
    cv::destroyAllWindows();
    
    std::cout << "Application ended." << std::endl;
    return 0;
}
