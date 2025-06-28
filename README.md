# 카메라 기반 PX4 정밀착륙 구성
---
1. 기존 패키지 대비 변경점은 다음과 같음
   - OpenCV 4.10 에서 OpenCV 4.5.4로 리팩토링 (Ubuntu 22.04 표준에 맞춤)
   - Aruco Marker 대신, 제 23회 로봇항공기 경연대회에 맞춰 AprilTag-36h10-96에 맞도록 리팩토링
   - px4-ros2-interface-lib에서 제공하는 ModeBase 기반 코드를 위 제약에 맞춰 리팩토링
   - ModeBase를 제거하고, 순수한 ROS2 Offboard 제어 기반 정밀착륙 코드 신규 작성 및 검증

2. 개발 환경
   - PX4-Autopilot v1.15 사용
   - QGC daily build 사용 (stable 버젼이랑 다름)
   - Ubuntu 22.04 LTS, ROS2 Humble, OpenCV 4.5.4, Gazebo Harmonic (SILS용) 사용

3. QGC에서의 사용법
```bash
# PX4와 QGC, uXRCE-DDS는 이미 켜져있다고 가정
# 1번째 터미널
cd tracktor-beam
source install/setup.bash
ros2 launch aruco_trakcer aruco_tracker.launch.py

# 2번째 터미널
cd tracktor-beam
source install/setup.bash
ros2 launch precision_land precision_land.launch.py
```

4. ROS2 Offboard 제어로서의 사용법
```bash
# PX4와 QGC, uXRCE-DDS는 이미 켜져있다고 가정
# 1번째 터미널
cd tracktor-beam
source install/setup.bash
ros2 launch aruco_trakcer aruco_tracker.launch.py

# 2번째 터미널
cd tracktor-beam
source install/setup.bash
ros2 launch precision_land precision_land_offboard.launch.py
```

5. 유의사항
- Gazebo에서 사용할거면, `aruco_tracker.launch.py`와 `ArucoTracker.cpp` 안에 있는 토픽 내용을 적절하게 수정해줄 것
