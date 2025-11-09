#pragma once
#include "../../lib/IMU/MPU6050.h"
#include <cmath>
#include <thread>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <vector>


class imuEstimator
{
public:
    imuEstimator();
    std::tuple<float, float, float> getOrientation();
private:
    void update();
    
    MPU6050 imu = MPU6050(0x68, false); // sudo i2cdetect -y 1 for the addr.
    float pitch, roll, yaw;
    float G; //constant for gravity, calculated during initialization.
    std::chrono::high_resolution_clock::time_point last_time;
    bool is_running = true;
};