#!/bin/bash

echo "=============================================="
echo "🚀 Scout Isaac ROS 워크스페이스 설정"
echo "=============================================="
echo ""
echo "📦 Hardware Stack:"
echo "  - Scout Mini (AgileX)"
echo "  - Jetson Orin NX"
echo "  - RealSense D435i (RGB-D + IMU)"
echo "  - XSENS MTi (9-axis IMU)"
echo ""
echo "🔧 Software Stack:"
echo "  ├─ Isaac ROS (Docker)"
echo "  │  ├─ Visual SLAM (cuVSLAM)"
echo "  │  └─ Nvblox (3D reconstruction)"
echo "  ├─ Robot Localization (EKF)"
echo "  ├─ Navigation2"
echo "  └─ Scout ROS2"
echo ""

# ========== Step 1: 기본 의존성 설치 ==========
echo "===== Step 1/6: 기본 의존성 설치 ====="
sudo apt-get update
sudo apt-get install -y \
  git-lfs \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  ca-certificates

# ========== Step 2: Docker 설치 ==========
echo ""
echo "===== Step 2/6: Docker 설치 ====="
if ! command -v docker &> /dev/null; then
    echo "Docker를 설치합니다..."
    
    # Docker GPG 키 추가
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    
    # Docker 저장소 추가
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Docker 설치
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    
    # 현재 사용자를 docker 그룹에 추가
    sudo usermod -aG docker $USER
    echo "✅ Docker 설치 완료. 로그아웃 후 다시 로그인해주세요."
else
    echo "✅ Docker가 이미 설치되어 있습니다."
fi

# ========== Step 3: NVIDIA Container Toolkit 설치 ==========
echo ""
echo "===== Step 3/6: NVIDIA Container Toolkit 설치 ====="
if ! command -v nvidia-ctk &> /dev/null; then
    echo "NVIDIA Container Toolkit을 설치합니다..."
    
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    
    # Docker용 NVIDIA runtime 설정
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    
    echo "✅ NVIDIA Container Toolkit 설치 완료"
else
    echo "✅ NVIDIA Container Toolkit이 이미 설치되어 있습니다."
fi

# ========== Step 4: 워크스페이스 생성 ==========
echo ""
echo "===== Step 4/6: 워크스페이스 생성 ====="
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev

# 환경 변수 설정
if ! grep -q "ISAAC_ROS_WS" ~/.bashrc; then
    echo 'export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"' >> ~/.bashrc
    echo "✅ ISAAC_ROS_WS 환경 변수 추가"
fi
export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"

# .repos 파일 생성 (Non-Isaac ROS 패키지들)
cat > src/scout_robot.repos <<'EOF'
repositories:
  # ========== Scout Robot Hardware ==========
  scout_ros2:
    type: git
    url: https://github.com/agilexrobotics/scout_ros2.git
    version: humble
  
  ugv_sdk:
    type: git
    url: https://github.com/agilexrobotics/ugv_sdk.git
    version: main
  
  # ========== Sensors ==========
  realsense_ros:
    type: git
    url: https://github.com/IntelRealSense/realsense-ros.git
    version: ros2-development
  
  xsens_mti_driver:
    type: git
    url: https://github.com/DEMCON/ros2_xsens_mti_driver.git
    version: main
  
  # ========== Localization & Navigation ==========
  robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: humble-devel
  
  navigation2:
    type: git
    url: https://github.com/ros-navigation/navigation2.git
    version: humble
EOF

# ========== Step 5: Isaac ROS 설치 ==========
echo ""
echo "===== Step 5/6: Isaac ROS 설치 ====="

# Isaac ROS Common 클론
cd ${ISAAC_ROS_WS}/src
if [ ! -d "isaac_ros_common" ]; then
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    echo "✅ isaac_ros_common 클론 완료"
fi

# Isaac ROS Visual SLAM 클론
if [ ! -d "isaac_ros_visual_slam" ]; then
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
    echo "✅ isaac_ros_visual_slam 클론 완료"
fi

# Isaac ROS Nvblox 클론
if [ ! -d "isaac_ros_nvblox" ]; then
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
    echo "✅ isaac_ros_nvblox 클론 완료"
fi

# 나머지 패키지들 다운로드
echo "나머지 패키지들을 다운로드합니다..."
cd ${ISAAC_ROS_WS}
vcs import src < src/scout_robot.repos

# RealSense 버전 호환성
cd src/realsense_ros
git checkout 4.54.1
cd ${ISAAC_ROS_WS}

# ========== Step 6: 통합 패키지 생성 ==========
echo ""
echo "===== Step 6/6: 통합 패키지 생성 ====="
cd ${ISAAC_ROS_WS}/src

# Scout Isaac 패키지 생성
if [ ! -d "scout_isaac" ]; then
    ros2 pkg create --build-type ament_python scout_isaac \
      --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros
fi

# package.xml 생성
cat > scout_isaac/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <n>scout_isaac</n>
  <version>0.1.0</version>
  <description>Scout Mini Robot with Isaac ROS Integration</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  
  <depend>scout_msgs</depend>
  <depend>scout_base</depend>
  
  <exec_depend>ros-humble-realsense2-camera</exec_depend>
  <depend>xsens_mti_driver</depend>
  <exec_depend>ros-humble-robot-localization</exec_depend>
  <exec_depend>ros-humble-navigation2</exec_depend>
  <exec_depend>ros-humble-nav2-bringup</exec_depend>
  
  <depend>robot_state_publisher</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Launch 디렉토리 및 설정 파일
mkdir -p scout_isaac/launch scout_isaac/config

# Launch 파일 생성
cat > scout_isaac/launch/scout_isaac_bringup.launch.py <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Scout 베이스
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
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_gyro': True,
                'enable_accel': True
            }]
        ),
        
        # Xsens IMU
        Node(
            package='xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_imu',
            output='screen'
        ),
        
        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf.yaml']
        ),
    ])
EOF

# EKF 설정
cat > scout_isaac/config/ekf.yaml <<'EOF'
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    
    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    
    odom1: /visual_slam/tracking/odometry
    odom1_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
