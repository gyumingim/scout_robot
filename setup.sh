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
