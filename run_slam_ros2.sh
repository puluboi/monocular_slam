#!/bin/bash

echo "Starting SLAM with ROS 2 RViz2 Visualization (Docker)"
echo "======================================================"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker not found! Please install Docker first."
    exit 1
fi

# Set up X11 forwarding for RViz2
echo "Setting up X11 forwarding for RViz2..."
xhost +local:docker

# Build Docker image if it doesn't exist
echo "Building Docker image (this may take a while the first time)..."
docker build -f Dockerfile.ros2 -t slam_ros2:latest .

if [ $? -ne 0 ]; then
    echo "Docker build failed! Check errors above."
    exit 1
fi

echo ""
echo "Starting Docker container with ROS 2..."
echo ""

# Run Docker container with:
# - X11 forwarding for RViz2
# - Privileged mode for I2C and camera access
# - Network host mode for ROS communication
# - Volume mount for hot-reload
docker run -it --rm \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v $(pwd):/ros2_ws/src/monocular_slam \
    slam_ros2:latest \
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd /ros2_ws && \
        colcon build --symlink-install && \
        source install/setup.bash && \
        echo '' && \
        echo 'ROS 2 SLAM node starting...' && \
        echo 'Topics being published:' && \
        echo '  - /slam/pointcloud (sensor_msgs/PointCloud2)' && \
        echo '  - /slam/camera_pose (geometry_msgs/PoseStamped)' && \
        echo '  - /slam/trajectory (nav_msgs/Path)' && \
        echo '  - /slam/features (sensor_msgs/Image)' && \
        echo '' && \
        echo 'To view in RViz2, run in another terminal:' && \
        echo '  docker exec -it \$(docker ps -q -f ancestor=slam_ros2:latest) bash -c \"source /opt/ros/jazzy/setup.bash && rviz2\"' && \
        echo '' && \
        ros2 run monocular_slam slam_node
    "

# Clean up X11 permissions
xhost -local:docker
