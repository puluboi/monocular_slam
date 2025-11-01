# ROS 2 RViz2 Visualization for Monocular SLAM

This replaces the non-working OpenCV viz 3D visualizer with ROS 2 RViz2, which works properly on Raspberry Pi.

**Note:** Since ROS 2 Humble/Jazzy packages have dependency conflicts with Debian 12 (Bookworm), we use **Docker** to run ROS 2 in an Ubuntu container.

## Prerequisites

### 1. Install Docker

If Docker is not installed:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

**Log out and log back in** for group changes to take effect.

### 2. Enable X11 Forwarding

For RViz2 GUI to work from Docker:

```bash
xhost +local:docker
```

Add this to your `~/.bashrc` to make it permanent:

```bash
echo 'xhost +local:docker > /dev/null 2>&1' >> ~/.bashrc
```

## Running the SLAM System with Visualization

### Quick Start (Recommended)

```bash
cd /home/puluboi/monocular_slam
./run_slam_ros2.sh
```

This will:
1. Build the Docker image with ROS 2 Jazzy (first time only - takes ~10-15 minutes)
2. Launch the SLAM node inside Docker
3. Publish ROS topics for visualization

### Opening RViz2

**In a separate terminal** (while SLAM node is running):

```bash
cd /home/puluboi/monocular_slam
./launch_rviz.sh
```

This opens RViz2 with the pre-configured visualization showing:
- 3D point cloud (colored by height)
- Camera trajectory (green path)
- Current camera pose (red axes)
- Feature tracking image

## Alternative: Manual Docker Commands

If you prefer manual control:

**Terminal 1 - Build and run SLAM node:**
```bash
# Build Docker image
docker build -f Dockerfile.ros2 -t slam_ros2:latest .

# Run SLAM container
xhost +local:docker
docker run -it --rm \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v $(pwd):/ros2_ws/src/monocular_slam \
    slam_ros2:latest \
    bash

# Inside container:
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run monocular_slam slam_node
```

**Terminal 2 - Launch RViz2:**
```bash
# Get container ID
docker ps

# Execute RViz2 in running container
docker exec -it <container_id> bash

# Inside container:
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
rviz2 -d /ros2_ws/src/monocular_slam/rviz_config.rviz
```

## Published Topics

The SLAM node publishes the following topics:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/slam/pointcloud` | `sensor_msgs/PointCloud2` | 3D point cloud from triangulation |
| `/slam/camera_pose` | `geometry_msgs/PoseStamped` | Current camera pose (position + orientation) |
| `/slam/trajectory` | `nav_msgs/Path` | Camera trajectory over time |
| `/slam/features` | `sensor_msgs/Image` | Feature tracking visualization image |

## Monitoring Topics

Check if topics are being published:

```bash
ros2 topic list
ros2 topic echo /slam/pointcloud
ros2 topic hz /slam/pointcloud
```

## RViz2 Configuration

The RViz2 configuration (`rviz_config.rviz`) includes:

- **Grid**: Reference grid in the world frame
- **PointCloud2**: 3D points colored by height (blue=low, red=high)
- **Path**: Green trajectory line showing camera movement
- **Image**: Feature tracking visualization
- **PoseStamped**: Red axes showing current camera pose

### Adjusting the View

In RViz2:
1. Use middle mouse button to rotate view
2. Scroll wheel to zoom
3. Shift + middle mouse to pan
4. Click "Reset" in Views panel to reset camera

## Troubleshooting

### No point cloud visible

1. Check if SLAM node is running:
   ```bash
   ros2 node list
   ```

2. Check if topics are publishing:
   ```bash
   ros2 topic hz /slam/pointcloud
   ```

3. Ensure there are enough features detected (move the camera to get features)

### RViz2 crashes or won't start

```bash
# Check ROS 2 installation
ros2 doctor

# Reinstall RViz2
sudo apt install --reinstall ros-jazzy-rviz2
```

### Build errors

```bash
# Clean build
cd /home/puluboi/monocular_slam
rm -rf build install log
colcon build --symlink-install
```

### Camera calibration

The default camera matrix in `slam_node.cpp` is:
```cpp
cv::Mat K = (cv::Mat_<double>(3,3) << 
    400, 0, 320,
    0, 400, 240,
    0, 0, 1);
```

For better accuracy, calibrate your camera and update these values.

## Why ROS 2 Instead of OpenCV viz?

OpenCV's viz module uses VTK and OpenGL shaders that don't work properly on Raspberry Pi's GPU drivers, causing errors like:

```
ERROR: In vtkShaderProgram.cxx
vtkShaderProgram error: syntax error, unexpected NEW_IDENTIFIER
```

ROS 2's RViz2 uses a different rendering pipeline that works correctly on ARM platforms like Raspberry Pi.

## Next Steps

1. **Camera Calibration**: Use `camera_calibration` package for accurate intrinsics
2. **IMU Integration**: Add IMU data fusion for better pose estimation
3. **Loop Closure**: Implement place recognition and loop closure
4. **Mapping**: Build and save 3D maps
5. **Recording**: Use `ros2 bag` to record data for offline processing

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [RViz2 User Guide](https://github.com/ros2/rviz/blob/jazzy/README.md)
- [sensor_msgs/PointCloud2](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/PointCloud2.html)
