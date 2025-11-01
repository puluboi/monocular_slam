#!/bin/bash

echo "Starting SLAM ROS 2 Node with RViz2 Visualization"
echo "=================================================="
echo ""

# Enable X11 forwarding; for the docker to be able to show visuals
xhost +local:docker > /dev/null 2>&1

# Run Docker container with RViz2
# there was a lot of difficulty with getting the code to access the cameras, but luckily in the end coPilot managed to pull it
sudo docker run -it --rm --privileged \
    --name slam_ros2_container \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e LIBCAMERA_IPA_MODULE_PATH=/usr/lib/aarch64-linux-gnu/libcamera/ipa \
    -e LIBCAMERA_PIPELINE_MODULE_PATH=/usr/lib/aarch64-linux-gnu/libcamera0.5/pipeline \
    -e LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/libcamera0.5:/usr/lib/aarch64-linux-gnu/libcamera:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera.so.0.5.2,target=/usr/lib/aarch64-linux-gnu/libcamera.so.0.5.2,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera.so,target=/usr/lib/aarch64-linux-gnu/libcamera.so,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera-base.so.0.5.2,target=/usr/lib/aarch64-linux-gnu/libcamera-base.so.0.5.2,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera-base.so,target=/usr/lib/aarch64-linux-gnu/libcamera-base.so,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera0.5,target=/usr/lib/aarch64-linux-gnu/libcamera0.5,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera,target=/usr/lib/aarch64-linux-gnu/libcamera,readonly \
    --mount type=bind,source=/usr/share/libcamera,target=/usr/share/libcamera,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libpisp.so.1,target=/usr/lib/aarch64-linux-gnu/libpisp.so.1,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libpisp.so.1.2.1,target=/usr/lib/aarch64-linux-gnu/libpisp.so.1.2.1,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libyuv.so.0,target=/usr/lib/aarch64-linux-gnu/libyuv.so.0,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libyuv.so.0.0.1857,target=/usr/lib/aarch64-linux-gnu/libyuv.so.0.0.1857,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libboost_log.so.1.74.0,target=/usr/lib/aarch64-linux-gnu/libboost_log.so.1.74.0,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.74.0,target=/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.74.0,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.74.0,target=/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.74.0,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libjpeg.so.62,target=/usr/lib/aarch64-linux-gnu/libjpeg.so.62,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libjpeg.so.62.3.0,target=/usr/lib/aarch64-linux-gnu/libjpeg.so.62.3.0,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera.so.0.5,target=/usr/lib/aarch64-linux-gnu/libcamera.so.0.5,readonly \
    --mount type=bind,source=/usr/lib/aarch64-linux-gnu/libcamera-base.so.0.5,target=/usr/lib/aarch64-linux-gnu/libcamera-base.so.0.5,readonly \
    --mount type=bind,source=/usr/share/libpisp,target=/usr/share/libpisp,readonly \
    --mount type=bind,source=/run/udev,target=/run/udev \
    -v $(pwd):/ros2_ws/src/monocular_slam \
    slam_ros2:latest \
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd /ros2_ws && \
        echo '🔨 Rebuilding workspace...' && \
        colcon build --packages-select monocular_slam --symlink-install && \
        source /ros2_ws/install/setup.bash && \
        echo '' && \
        echo '✅ Build complete!' && \
        echo 'Starting SLAM node in background...' && \
        ros2 run monocular_slam slam_node &
        SLAM_PID=\$! && \
        sleep 2 && \
        echo 'Launching RViz2...' && \
        rviz2 -d /ros2_ws/src/monocular_slam/rviz_config.rviz && \
        kill \$SLAM_PID
    "