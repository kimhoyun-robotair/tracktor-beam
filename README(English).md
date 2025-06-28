# Camera-Based PX4 Precision Landing Configuration
---
1. Changes Compared to the Original Package
    - Refactored from OpenCV 4.10 to OpenCV 4.5.4 (to match Ubuntu 22.04 standard)
    - Swapped out Aruco Marker for AprilTag-36h10-96, as required by the 23rd Aerial Robotics Contest
    - Refactored the ModeBase-based code from px4-ros2-interface-lib to meet the above constraints
    - Removed ModeBase entirely and wrote & validated a brand-new precision-landing implementation based purely on ROS 2 Offboard control

2. Development Environment
   - PX4-Autopilot v1.15
   - QGroundControl daily build (not the stable release)
   - Ubuntu 22.04 LTS, ROS 2 Humble, OpenCV 4.5.4, Gazebo Harmonic (for SILS)

3. How to Run via QGC
```bash
# (Assuming PX4, QGC, and uXRCE-DDS are already running)
# ─── Terminal 1 ───
cd tracktor-beam
source install/setup.bash
ros2 launch aruco_trakcer aruco_tracker.launch.py

# ─── Terminal 2 ───
cd tracktor-beam
source install/setup.bash
ros2 launch precision_land precision_land.launch.py
```

4. How to Run as ROS 2 Offboard Control
```bash
# (Assuming PX4, QGC, and uXRCE-DDS are already running)
# ─── Terminal 1 ───
cd tracktor-beam
source install/setup.bash
ros2 launch aruco_trakcer aruco_tracker.launch.py

# ─── Terminal 2 ───
cd tracktor-beam
source install/setup.bash
ros2 launch precision_land precision_land_offboard.launch.py
```

5. Notes

    If you plan to run this in Gazebo, be sure to adjust the topic names inside both aruco_tracker.launch.py and ArucoTracker.cpp accordingly.