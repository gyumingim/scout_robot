# ========== Docker 컨테이너 실행 스크립트 생성 ==========
cat > ${ISAAC_ROS_WS}/run_isaac_container.sh <<'EOF'
#!/bin/bash

# Isaac ROS Docker 컨테이너 실행
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
EOF

chmod +x ${ISAAC_ROS_WS}/run_isaac_container.sh

# ========== 완료 메시지 ==========
echo ""
echo "=============================================="
echo "✅ Scout Isaac ROS 워크스페이스 설치 완료!"
echo "=============================================="
echo ""
echo "📂 워크스페이스: ${ISAAC_ROS_WS}"
echo ""
echo "🐳 다음 단계:"
echo ""
echo "1️⃣ Docker 그룹 권한 적용 (필수, 최초 1회)"
echo "   newgrp docker"
echo "   (또는 로그아웃 후 재로그인)"
echo ""
echo "2️⃣ Isaac ROS Docker 컨테이너 실행"
echo "   cd \${ISAAC_ROS_WS}/src/isaac_ros_common"
echo "   ./scripts/run_dev.sh"
echo ""
echo "3️⃣ 컨테이너 내부에서 빌드"
echo "   cd /workspaces/isaac_ros-dev"
echo "   rosdep install --from-paths src --ignore-src -y"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "4️⃣ 로봇 실행"
echo "   ros2 launch scout_isaac scout_isaac_bringup.launch.py"
echo ""
echo "📝 Isaac ROS 패키지들:"
echo "   - isaac_ros_visual_slam (cuVSLAM)"
echo "   - isaac_ros_nvblox (3D reconstruction)"
echo "   ※ 이 패키지들은 Docker 내에서 빌드됩니다"
echo ""
echo "⚠️  중요:"
echo "   - Isaac ROS는 NVIDIA GPU 필수"
echo "   - Jetson Orin NX에 최적화됨"
echo "   - 모든 Isaac ROS 작업은 Docker 컨테이너 내에서 수행"
echo ""
