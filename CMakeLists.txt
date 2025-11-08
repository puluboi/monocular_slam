cmake_minimum_required(VERSION 3.10)
project(monocular_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Threads REQUIRED)

set(SLAM_CORE_SOURCES
    src/estimator/featureTracker.cpp
    lib/IMU/MPU6050.cpp
    src/camera/camera.cpp
)

set(SLAM_CORE_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib
)

if(ament_cmake_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)

    add_library(slam_core STATIC ${SLAM_CORE_SOURCES})
    target_include_directories(slam_core
        PUBLIC
            ${SLAM_CORE_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${LIBCAMERA_INCLUDE_DIRS}
    )
    ament_target_dependencies(slam_core
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        cv_bridge
    )
    target_link_libraries(slam_core
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
        Threads::Threads
        i2c
    )

    add_executable(slam_node
        src/ros2/slam_node.cpp
        src/ros2/PointCloudPublisher.cpp
    )
    target_include_directories(slam_node PRIVATE ${SLAM_CORE_INCLUDE_DIRS})
    ament_target_dependencies(slam_node
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        cv_bridge
    )
    target_link_libraries(slam_node slam_core)


    install(TARGETS slam_node
        DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()
else()
    find_package(OpenCV REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)

    add_library(slam_core STATIC ${SLAM_CORE_SOURCES})
    target_include_directories(slam_core
        PUBLIC
            ${SLAM_CORE_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${LIBCAMERA_INCLUDE_DIRS}
    )
    target_link_libraries(slam_core
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
        Threads::Threads
        i2c
    )

    add_executable(monocular_slam src/main.cpp)
    target_include_directories(monocular_slam PRIVATE ${SLAM_CORE_INCLUDE_DIRS})
    target_link_libraries(monocular_slam PRIVATE slam_core)
    
endif()
