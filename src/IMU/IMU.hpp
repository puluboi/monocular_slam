#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>
#include <string>

class MPU6050 {
private:
    int i2c_fd_;
    uint8_t device_address_;
    bool is_initialized_;
    
    // Calibration offsets
    float accel_offset_x_, accel_offset_y_, accel_offset_z_;
    float gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;
    
    // Gravity estimation for gravity compensation
    float gravity_x_, gravity_y_, gravity_z_;
    float alpha_; // Low-pass filter coefficient for gravity estimation

public:
    struct IMUData {
        float accel_x, accel_y, accel_z;  // m/s²
        float gyro_x, gyro_y, gyro_z;     // rad/s
        float temperature;                 // °C
        uint64_t timestamp;               // microseconds
    };

    MPU6050(uint8_t address = 0x68);
    ~MPU6050();
    
    bool initialize(const std::string& i2c_device = "/dev/i2c-1");
    bool calibrate(int samples = 1000);
    bool readData(IMUData& data);
    void release();
    
    // Configuration methods
    bool setAccelRange(int range);    // ±2, ±4, ±8, ±16 g
    bool setGyroRange(int range);     // ±250, ±500, ±1000, ±2000 °/s
    bool setFilterBandwidth(int bw);  // Low-pass filter

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, int length);
};

#endif // IMU_HPP