#include "imuEstimator.hpp"
#include <numeric>


imuEstimator::imuEstimator()
{

    int loop_size=100;
    std::vector<float> arx_, ary_, arz_, mag_;
    arx_.reserve(loop_size);
    ary_.reserve(loop_size);
    arz_.reserve(loop_size);
    for (size_t i = 0; i < loop_size; i++)
    {

        float arx, ary, arz, mag;
        imu.getAccelRaw(&ary,&arx,&arz); // In the curr physical setup, x is where y would be and y is opposite of where x would be
        arx_.push_back(arx);
        ary_.push_back(-ary);
        arz_.push_back(arz);
        mag = sqrt(arx*arx + ary*ary + arz*arz);
        mag_.push_back(mag);
        usleep(10000);
    }
    float sum_arx = std::accumulate(arx_.begin(), arx_.end(), 0.0f);
    float sum_ary = std::accumulate(ary_.begin(), ary_.end(), 0.0f);
    float sum_arz = std::accumulate(arz_.begin(), arz_.end(), 0.0f);
    float sum_mag = std::accumulate(mag_.begin(), mag_.end(), 0.0f);

    float avg_arx = sum_arx / arx_.size();
    float avg_ary = sum_ary / ary_.size();
    float avg_arz = sum_arz / arz_.size();
    G = sum_mag /mag_.size();
    // Clamp the value to valid range for asin [-1, 1]
    float arx_normalized = std::max(-1.0f, std::min(1.0f, avg_arx / G));
    
    pitch = asin(-arx_normalized);
    roll = atan(avg_ary/avg_arz);
    yaw = 0;
     
    std::cout<<"roll: "<< roll<< " pitch: " << pitch<<" Gravity: "<< G << std::endl;
    

    last_time = std::chrono::high_resolution_clock::now();
    std::thread(&imuEstimator::update, this).detach();
}

void imuEstimator::update()
{
    while (is_running)  
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;  // Move this AFTER dt calculation but BEFORE next iteration
        
        float grx, gry, grz;
        
        imu.getGyro(&grx, &gry, &grz);
        
        // Integrate gyroscope data (angular velocities in rad/s)
        pitch = fmod(pitch - gry * dt, 2*M_PI); // fmod to wrap the angles( roll % 360)
        roll = fmod(roll - grz * dt, 2*M_PI);
        yaw = fmod(yaw - grx * dt, 2*M_PI);
        
        usleep(5000); // ~200Hz update rate (5ms delay)
    }
}
std::tuple<float, float, float> imuEstimator::getOrientation()
{
    return std::make_tuple(roll, pitch, yaw);
}
