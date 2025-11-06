# monocular_slam
aim is to build a system capable of 3D slam using a camera &amp; imu on a rpi5.

18.9.2025
1. Task: Get IMU running on C++ code.
22.9.2025
2. Task: Get basic feature detection working: https://learnopencv.com/monocular-slam-in-python/
25.9.2025
3. Task: Get feature matching working 
24.10.2025
4. Task: Triangulate the camera positions
 --- integrate into rviz
5. 1.11.2025
    - I got the triangulated 3d points to be published into the ros visualization.
    - A real challenge was operating the Docker environment
        - it was a pain in the ass to get the libcamera in the docker to have access to the real hardware camera connected to my raspberry.
    - Next issue is getting the IMU and feature tracker to work in sync and create a usable point cloud, now that it's visualized i can see it is not working so good at all.
6. 6.11.25
    - One problem seems to be that the pose1 and pose2 matrices are always initialized, when they should use the previous poses?