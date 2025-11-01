FROM --platform=linux/arm/v7 arm32v7/ubuntu:20.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for your SLAM system
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    libeigen3-dev \
    i2c-tools \
    libi2c-dev \
    python3-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy your source code (excluding build directory via .dockerignore)
COPY . .

# Create a Docker-compatible CMakeLists.txt that builds without libcamera dependency
RUN echo 'cmake_minimum_required(VERSION 3.10.0)\n\
project(monocular_slam VERSION 0.1.0 LANGUAGES C CXX)\n\
\n\
# Find required packages\n\
find_package(OpenCV REQUIRED)\n\
\n\
# Add IMU-only executable for Docker testing\n\
add_executable(monocular_slam_imu \n\
    src/IMU/IMU.cpp\n\
    docker/imu_test.cpp\n\
)\n\
\n\
# Include directories\n\
target_include_directories(monocular_slam_imu PRIVATE \n\
    src\n\
    ${OpenCV_INCLUDE_DIRS}\n\
)\n\
\n\
# Link libraries\n\
target_link_libraries(monocular_slam_imu PRIVATE \n\
    ${OpenCV_LIBS}\n\
)\n' > CMakeLists.docker.txt

# Create simple IMU test program
RUN mkdir -p docker && cat > docker/imu_test.cpp << 'EOF'
#include "IMU/IMU.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "=== Docker IMU Test ===" << std::endl;
    
    MPU6050 imu;
    
    if (!imu.initialize()) {
        std::cerr << "Failed to initialize IMU (expected in Docker without hardware)" << std::endl;
        std::cout << "IMU code compiled successfully!" << std::endl;
        return 0;
    }
    
    std::cout << "IMU initialized! Calibrating..." << std::endl;
    imu.calibrate(50);
    
    for(int i = 0; i < 10; i++) {
        MPU6050::IMUData data;
        if(imu.readData(data)) {
            std::cout << "Reading " << i+1 << ": Accel=["
                      << data.accel_x << ", " << data.accel_y << ", " << data.accel_z
                      << "] Gyro=[" << data.gyro_x << ", " << data.gyro_y << ", " << data.gyro_z
                      << "] Temp=" << data.temperature << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
EOF

# Build the Docker-compatible version  
RUN mv CMakeLists.txt CMakeLists.txt.orig && \
    mv CMakeLists.docker.txt CMakeLists.txt && \
    rm -rf build/ && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc)

# Set executable permissions
RUN chmod +x build/monocular_slam_imu

# Default command to run IMU test
CMD ["./build/monocular_slam_imu"]