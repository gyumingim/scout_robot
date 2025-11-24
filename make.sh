#!/bin/bash

# ROS2 워크스페이스 생성
mkdir -p ~/scout_robot_ws/src
cd ~/scout_robot_ws

# .repos 파일 생성 (VCS tools용 - GitHub 저장소 관리)
cat > src/scout_robot.repos <<'EOF'
repositories:
  # Scout Mini 제어 및 센서 신호 수신
  scout_ros2:
    type: git
    url: https://github.com/agilexrobotics/scout_ros2.git
    version: humble
  
  # Scout 연결 드라이버
  ugv_sdk:
    type: git
    url: https://github.com/agilexrobotics/ugv_sdk.git
    version: main
  
  # Navigation2 - 경로 생성
  navigation2:
    type: git
    url: https://github.com/ros-navigation/navigation2.git
    version: humble
  
  # 센서 퓨전 (Odom + IMU)
  robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: humble-devel
  
  # RTAB-Map - 지도 생성 핵심
  rtabmap:
    type: git
    url: https://github.com/introlab/rtabmap.git
    version: humble-devel
  
  # RTAB-Map ROS2 래퍼
  rtabmap_ros:
    type: git
    url: https://github.com/introlab/rtabmap_ros.git
    version: humble-devel
  
  # RealSense 카메라 드라이버
  realsense_ros:
    type: git
    url: https://github.com/IntelRealSense/realsense-ros.git
    version: ros2-development
  
  # Xsens IMU 드라이버
  xsens_mti_driver:
    type: git
    url: https://github.com/DEMCON/ros2_xsens_mti_driver.git
    version: main
EOF

# 통합 패키지 생성
cd src
ros2 pkg create --build-type ament_python scout_robot \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros

# package.xml에 모든 의존성 작성
cat > scout_robot/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>scout_robot</name>
  <version>0.0.1</version>
  <description>Scout Mini Robot Integration Package</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <!-- 기본 ROS2 의존성 -->
  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <!-- Scout 관련 -->
  <depend>scout_msgs</depend>
  <depend>scout_base</depend>
  
  <!-- Navigation2 -->
  <exec_depend>ros-humble-navigation2</exec_depend>
  <exec_depend>ros-humble-nav2-bringup</exec_depend>
  
  <!-- Robot Localization (센서 퓨전) -->
  <exec_depend>ros-humble-robot-localization</exec_depend>
  
  <!-- RTAB-Map -->
  <exec_depend>ros-humble-rtabmap</exec_depend>
  <exec_depend>ros-humble-rtabmap-ros</exec_depend>
  
  <!-- RealSense 카메라 -->
  <exec_depend>ros-humble-realsense2-camera</exec_depend>
  <exec_depend>ros-humble-realsense2-description</exec_depend>
  
  <!-- IMU 관련 -->
  <depend>xsens_mti_driver</depend>
  
  <!-- URDF 및 로봇 모델 -->
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>
  
  <!-- 시각화 -->
  <exec_depend>ros-humble-rviz2</exec_depend>
  
  <!-- 추가 유틸리티 -->
  <exec_depend>ros-humble-image-transport</exec_depend>
  <exec_depend>ros-humble-compressed-image-transport</exec_depend>
  <exec_depend>ros-humble-depth-image-proc</exec_depend>

  <!-- 빌드 도구 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- 테스트 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# launch 디렉토리 생성
mkdir -p scout_robot/launch
mkdir -p scout_robot/config

# 기본 launch 파일 생성
cat > scout_robot/launch/scout_bringup.launch.py <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Scout 베이스 컨트롤러
        Node(
            package='scout_base',
            executable='scout_base_node',
            name='scout_base',
            output='screen'
        ),
        
        # RealSense 카메라
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen'
        ),
        
        # Xsens IMU
        Node(
            package='xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_imu',
            output='screen'
        ),
        
        # Robot Localization (센서 퓨전)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf.yaml']
        ),
    ])
EOF

cd ~/scout_robot_ws

# VCS tools로 소스 가져오기
echo "===== 소스 코드 다운로드 ====="
vcs import src < src/scout_robot.repos

# RealSense 버전 호환성 문제 해결
echo "===== RealSense 버전 호환성 설정 ====="
cd src/realsense_ros
git checkout 4.54.1  # 2.56.x SDK와 호환되는 버전
cd ~/scout_robot_ws

# rosdep으로 모든 의존성 자동 설치
echo "===== 의존성 설치 ====="
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

# 빌드
echo "===== 빌드 ====="
colcon build --symlink-install

# 환경 설정
echo ""
echo "====================================="
echo "✅ Scout Robot 워크스페이스 설치 완료!"
echo "====================================="
echo ""
echo "다음 명령어로 워크스페이스를 활성화하세요:"
echo "  source ~/scout_robot_ws/install/setup.bash"
echo ""
echo "로봇 실행:"
echo "  ros2 launch scout_robot scout_bringup.launch.py"
echo ""
EOF
