#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "camera/camera.hpp"
#include "IMU/IMU.hpp"

int main(int, char**) {
    std::cout << "Monocular SLAM - Camera Stream" << std::endl;

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
    
    // Initialize camera
    std::cout << "Initializing camera..." << std::endl;
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
        if(imu_init) {
            MPU6050::IMUData imu_data;
            if (imu.readData(imu_data)) {
                // Convert acceleration to g-forces (1g = 9.81 m/s²)
                float accel_x_g = imu_data.accel_x / 9.81f;
                float accel_y_g = imu_data.accel_y / 9.81f;
                float accel_z_g = imu_data.accel_z / 9.81f;
                
                // Calculate total acceleration magnitude in g
                float total_g = sqrt(accel_x_g*accel_x_g + accel_y_g*accel_y_g + accel_z_g*accel_z_g);
                
                std::cout << "Accel (g): [" 
                          << std::fixed << std::setprecision(2)
                          << accel_x_g << ", " 
                          << accel_y_g << ", " 
                          << accel_z_g << "] "
                          << "Total: " << total_g << "g | "
                          << "Gyro (°/s): [" 
                          << std::setprecision(1)
                          << imu_data.gyro_x * 180.0f/M_PI << ", " 
                          << imu_data.gyro_y * 180.0f/M_PI << ", " 
                          << imu_data.gyro_z * 180.0f/M_PI << "] | "
                          << "Temp: " << std::setprecision(1) << imu_data.temperature << "°C"
                          << std::endl;
            } else {
                std::cerr << "Failed to read IMU data" << std::endl;
            }
        }
    }
    
    // Cleanup
    cv::destroyAllWindows();
    camera.stopCapture();
    camera.release();
    
    std::cout << "Application ended." << std::endl;
    return 0;
}
