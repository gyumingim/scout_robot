# VCS tools로 소스 가져오기
echo "===== 소스 코드 다운로드 ====="
vcs import src < src/scout_isaac.repos

# RealSense 버전 호환성 문제 해결
echo "===== RealSense 버전 호환성 설정 ====="
cd src/realsense_ros
git checkout 4.54.1
cd ~/scout_isaac_ws

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
echo "=============================================="
echo "✅ Scout Isaac ROS 워크스페이스 설치 완료!"
echo "=============================================="
echo ""
echo "🤖 Hardware Stack:"
echo "  - Scout Mini (AgileX)"
echo "  - Jetson Orin NX"
echo "  - RealSense D435i (RGB-D + IMU)"
echo "  - XSENS MTi (9-axis IMU)"
echo ""
echo "🚀 Software Stack:"
echo "  ├─ Perception:"
echo "  │  ├─ Isaac ROS Visual SLAM (cuVSLAM)"
echo "  │  └─ Isaac ROS Nvblox (3D reconstruction)"
echo "  ├─ Localization:"
echo "  │  └─ robot_localization (EKF fusion)"
echo "  ├─ Navigation:"
echo "  │  └─ Navigation2"
echo "  └─ Control:"
echo "     └─ scout_ros2 + ugv_sdk"
echo ""
echo "다음 명령어로 워크스페이스를 활성화하세요:"
echo "  source ~/scout_isaac_ws/install/setup.bash"
echo ""
echo "로봇 실행:"
echo "  ros2 launch scout_isaac scout_isaac_bringup.launch.py"
echo ""
echo "⚠️  참고사항:"
echo "  - Isaac ROS는 NVIDIA GPU가 필수입니다"
echo "  - Jetson Orin NX에서 최적 성능을 제공합니다"
echo "  - Isaac Sim 연동을 위해서는 별도 설정이 필요합니다"
echo ""
