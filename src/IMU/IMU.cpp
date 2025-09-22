#include "IMU.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>

// MPU6050 Register addresses
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

// Scale factors
#define ACCEL_SCALE_2G          16384.0f
#define ACCEL_SCALE_4G          8192.0f
#define ACCEL_SCALE_8G          4096.0f
#define ACCEL_SCALE_16G         2048.0f

#define GYRO_SCALE_250          131.0f
#define GYRO_SCALE_500          65.5f
#define GYRO_SCALE_1000         32.8f
#define GYRO_SCALE_2000         16.4f

MPU6050::MPU6050(uint8_t address) 
    : i2c_fd_(-1), device_address_(address), is_initialized_(false),
      accel_offset_x_(0), accel_offset_y_(0), accel_offset_z_(0),
      gyro_offset_x_(0), gyro_offset_y_(0), gyro_offset_z_(0),
      gyro_bias_x_(0), gyro_bias_y_(0), gyro_bias_z_(0),
      last_temperature_(0), still_count_(0),
      gravity_x_(0), gravity_y_(0), gravity_z_(9.81f), alpha_(0.98f),
      roll_(0), pitch_(0), yaw_(0), orientation_initialized_(false),
      last_timestamp_(0) {
}

MPU6050::~MPU6050() {
    release();
}

bool MPU6050::initialize(const std::string& i2c_device) {
    // Open I2C device
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "Failed to open I2C device: " << i2c_device << std::endl;
        return false;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, device_address_) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // Wake up the MPU6050 (reset sleep mode)
    if (!writeRegister(MPU6050_PWR_MGMT_1, 0x00)) {
        std::cerr << "Failed to wake up MPU6050" << std::endl;
        release();
        return false;
    }

    // Set sample rate to 1kHz
    if (!writeRegister(MPU6050_SMPLRT_DIV, 0x07)) {
        std::cerr << "Failed to set sample rate" << std::endl;
        release();
        return false;
    }

    // Configure accelerometer (±2g)
    if (!writeRegister(MPU6050_ACCEL_CONFIG, 0x00)) {
        std::cerr << "Failed to configure accelerometer" << std::endl;
        release();
        return false;
    }

    // Configure gyroscope (±250°/s)
    if (!writeRegister(MPU6050_GYRO_CONFIG, 0x00)) {
        std::cerr << "Failed to configure gyroscope" << std::endl;
        release();
        return false;
    }

    // Set digital low pass filter
    if (!writeRegister(MPU6050_CONFIG, 0x03)) {
        std::cerr << "Failed to set digital filter" << std::endl;
        release();
        return false;
    }

    is_initialized_ = true;
    std::cout << "MPU6050 initialized successfully" << std::endl;
    return true;
}

bool MPU6050::calibrate(int samples) {
    if (!is_initialized_) {
        std::cerr << "MPU6050 not initialized" << std::endl;
        return false;
    }

    std::cout << "Calibrating MPU6050... Keep the sensor still!" << std::endl;

    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < samples; i++) {
        IMUData data;
        if (readData(data)) {
            accel_sum_x += data.accel_x;
            accel_sum_y += data.accel_y;
            accel_sum_z += data.accel_z; // Subtract gravity
            gyro_sum_x += data.gyro_x;
            gyro_sum_y += data.gyro_y;
            gyro_sum_z += data.gyro_z;
        }
        usleep(1000); // 1ms delay
    }

    accel_offset_x_ = accel_sum_x / samples;
    accel_offset_y_ = accel_sum_y / samples;
    accel_offset_z_ = accel_sum_z / samples;
    gyro_offset_x_ = gyro_sum_x / samples;
    gyro_offset_y_ = gyro_sum_y / samples;
    gyro_offset_z_ = gyro_sum_z / samples;

    std::cout << "Calibration complete!" << std::endl;
    std::cout << "Accel offsets: " << accel_offset_x_ << ", " << accel_offset_y_ << ", " << accel_offset_z_ << std::endl;
    std::cout << "Gyro offsets: " << gyro_offset_x_ << ", " << gyro_offset_y_ << ", " << gyro_offset_z_ << std::endl;

    return true;
}

bool MPU6050::readData(IMUData& data) {
    if (!is_initialized_) {
        std::cerr << "MPU6050 not initialized" << std::endl;
        return false;
    }

    uint8_t buffer[14];
    
    // Read all sensor data in one go (14 bytes starting from ACCEL_XOUT_H)
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }

    // Get timestamp
    auto now = std::chrono::high_resolution_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    data.timestamp = timestamp.count();

    // Combine high and low bytes for accelerometer
    int16_t accel_x_raw = (buffer[0] << 8) | buffer[1];
    int16_t accel_y_raw = (buffer[2] << 8) | buffer[3];
    int16_t accel_z_raw = (buffer[4] << 8) | buffer[5];

    // Temperature
    int16_t temp_raw = (buffer[6] << 8) | buffer[7];

    // Combine high and low bytes for gyroscope
    int16_t gyro_x_raw = (buffer[8] << 8) | buffer[9];
    int16_t gyro_y_raw = (buffer[10] << 8) | buffer[11];
    int16_t gyro_z_raw = (buffer[12] << 8) | buffer[13];

        // Convert to physical units (raw acceleration including gravity)
    float raw_accel_x = (accel_x_raw / ACCEL_SCALE_2G) * 9.81f - accel_offset_x_;  // m/s²
    float raw_accel_y = (accel_y_raw / ACCEL_SCALE_2G) * 9.81f - accel_offset_y_;
    float raw_accel_z = (accel_z_raw / ACCEL_SCALE_2G) * 9.81f - accel_offset_z_;

    // Calculate gyroscope readings with initial calibration
    float raw_gyro_x = (gyro_x_raw / GYRO_SCALE_250) * (M_PI / 180.0f) - gyro_offset_x_;  // rad/s
    float raw_gyro_y = (gyro_y_raw / GYRO_SCALE_250) * (M_PI / 180.0f) - gyro_offset_y_;
    float raw_gyro_z = (gyro_z_raw / GYRO_SCALE_250) * (M_PI / 180.0f) - gyro_offset_z_;

    // Get temperature for drift compensation
    data.temperature = (temp_raw / 340.0f) + 36.53f;  // °C

    // Dynamic gyroscope bias correction
    // Check if device is still (low acceleration + low rotation)
    float accel_magnitude = sqrt(raw_accel_x*raw_accel_x + raw_accel_y*raw_accel_y + raw_accel_z*raw_accel_z);
    float gyro_magnitude = sqrt(raw_gyro_x*raw_gyro_x + raw_gyro_y*raw_gyro_y + raw_gyro_z*raw_gyro_z);
    
    bool is_still = (fabs(accel_magnitude - 9.81f) < 0.5f) && (gyro_magnitude < 0.05f); // 0.05 rad/s ≈ 3°/s
    
    if (is_still) {
        still_count_++;
        
        // After being still for a while, update bias estimates
        if (still_count_ > 50) { // ~50ms of stillness
            float bias_alpha = 0.001f; // Very slow adaptation
            gyro_bias_x_ = (1.0f - bias_alpha) * gyro_bias_x_ + bias_alpha * raw_gyro_x;
            gyro_bias_y_ = (1.0f - bias_alpha) * gyro_bias_y_ + bias_alpha * raw_gyro_y;
            gyro_bias_z_ = (1.0f - bias_alpha) * gyro_bias_z_ + bias_alpha * raw_gyro_z;
        }
    } else {
        still_count_ = 0; // Reset counter when moving
    }
    
    // Apply dynamic bias correction
    data.gyro_x = raw_gyro_x - gyro_bias_x_;
    data.gyro_y = raw_gyro_y - gyro_bias_y_;
    data.gyro_z = raw_gyro_z - gyro_bias_z_;

    // Simple approach: High-pass filter to remove DC component (gravity)
    // Initialize filter on first run
    static bool filter_init = false;
    static float hp_accel_x = 0, hp_accel_y = 0, hp_accel_z = 0;
    static float prev_raw_x = 0, prev_raw_y = 0, prev_raw_z = 0;
    
    if (!filter_init) {
        hp_accel_x = 0;
        hp_accel_y = 0; 
        hp_accel_z = 0;
        prev_raw_x = raw_accel_x;
        prev_raw_y = raw_accel_y;
        prev_raw_z = raw_accel_z;
        filter_init = true;
    }
    
    // High-pass filter: removes DC (gravity), keeps AC (motion)
    float alpha = 0.8f;  // Filter coefficient
    hp_accel_x = alpha * (hp_accel_x + raw_accel_x - prev_raw_x);
    hp_accel_y = alpha * (hp_accel_y + raw_accel_y - prev_raw_y);
    hp_accel_z = alpha * (hp_accel_z + raw_accel_z - prev_raw_z);
    
    prev_raw_x = raw_accel_x;
    prev_raw_y = raw_accel_y;
    prev_raw_z = raw_accel_z;
    
    data.accel_x = hp_accel_x;
    data.accel_y = hp_accel_y;
    data.accel_z = hp_accel_z;

    return true;
}

void MPU6050::release() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    is_initialized_ = false;
}

bool MPU6050::setAccelRange(int range) {
    if (!is_initialized_) {
        return false;
    }

    uint8_t config_value;
    switch (range) {
        case 2:  config_value = 0x00; break;
        case 4:  config_value = 0x08; break;
        case 8:  config_value = 0x10; break;
        case 16: config_value = 0x18; break;
        default:
            std::cerr << "Invalid accelerometer range: " << range << std::endl;
            return false;
    }

    return writeRegister(MPU6050_ACCEL_CONFIG, config_value);
}

bool MPU6050::setGyroRange(int range) {
    if (!is_initialized_) {
        return false;
    }

    uint8_t config_value;
    switch (range) {
        case 250:  config_value = 0x00; break;
        case 500:  config_value = 0x08; break;
        case 1000: config_value = 0x10; break;
        case 2000: config_value = 0x18; break;
        default:
            std::cerr << "Invalid gyroscope range: " << range << std::endl;
            return false;
    }

    return writeRegister(MPU6050_GYRO_CONFIG, config_value);
}

bool MPU6050::setFilterBandwidth(int bw) {
    if (!is_initialized_) {
        return false;
    }

    uint8_t config_value;
    switch (bw) {
        case 260: config_value = 0x00; break;
        case 184: config_value = 0x01; break;
        case 94:  config_value = 0x02; break;
        case 44:  config_value = 0x03; break;
        case 21:  config_value = 0x04; break;
        case 10:  config_value = 0x05; break;
        case 5:   config_value = 0x06; break;
        default:
            std::cerr << "Invalid filter bandwidth: " << bw << std::endl;
            return false;
    }

    return writeRegister(MPU6050_CONFIG, config_value);
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    
    if (write(i2c_fd_, buffer, 2) != 2) {
        std::cerr << "Failed to write register 0x" << std::hex << (int)reg << std::endl;
        return false;
    }
    
    return true;
}

bool MPU6050::readRegister(uint8_t reg, uint8_t& value) {
    if (write(i2c_fd_, &reg, 1) != 1) {
        std::cerr << "Failed to write register address" << std::endl;
        return false;
    }

    if (read(i2c_fd_, &value, 1) != 1) {
        std::cerr << "Failed to read register value" << std::endl;
        return false;
    }

    return true;
}

bool MPU6050::readRegisters(uint8_t reg, uint8_t* buffer, int length) {
    if (write(i2c_fd_, &reg, 1) != 1) {
        std::cerr << "Failed to write register address" << std::endl;
        return false;
    }

    if (read(i2c_fd_, buffer, length) != length) {
        std::cerr << "Failed to read register values" << std::endl;
        return false;
    }

    return true;
}
