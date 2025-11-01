#!/bin/bash

echo "Launching RViz2 in Docker Container"
echo "===================================="
echo ""

# Check if SLAM container is running
if ! docker ps | grep -q slam_ros2; then
    echo "Error: SLAM container is not running!"
    echo "Please start the SLAM node first with: ./run_slam_ros2.sh"
    exit 1
fi

# Get the SLAM container ID
CONTAINER_ID=$(docker ps -q -f ancestor=slam_ros2:latest)

echo "Starting RViz2 in SLAM container: $CONTAINER_ID"
echo ""

# Execute RViz2 in the running container
docker exec -it $CONTAINER_ID bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    rviz2 -d /ros2_ws/src/monocular_slam/rviz_config.rviz
"
