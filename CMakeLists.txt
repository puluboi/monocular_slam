cmake_minimum_required(VERSION 3.10.0)
project(monocular_slam VERSION 0.1.0 LANGUAGES C CXX)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcamera using pkg-config
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Add source files
add_executable(monocular_slam 
    src/main.cpp
    src/camera/camera.cpp
    src/IMU/IMU.cpp
    src/estimator/featureTracker.cpp
)

# Include directories
target_include_directories(monocular_slam PRIVATE 
    src/camera
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(monocular_slam PRIVATE 
    ${LIBCAMERA_LIBRARIES}
    ${OpenCV_LIBS}
)
include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
