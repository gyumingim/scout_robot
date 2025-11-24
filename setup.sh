#!/bin/bash

# Isaac ROS Scout Robot 워크스페이스 생성
mkdir -p ~/scout_isaac_ws/src
cd ~/scout_isaac_ws

# .repos 파일 생성 (VCS tools용)
cat > src/scout_isaac.repos <<'EOF'
repositories:
  # ========== Scout Robot Hardware ==========
  # Scout Mini 제어 및 센서 신호
  scout_ros2:
    type: git
    url: https://github.com/agilexrobotics/scout_ros2.git
    version: humble
  
  # Scout 연결 드라이버
  ugv_sdk:
    type: git
    url: https://github.com/agilexrobotics/ugv_sdk.git
    version: main
  
  # ========== Isaac ROS Perception ==========
  # Isaac ROS 공통 유틸리티
  isaac_ros_common:
    type: git
    url: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    version: main
  
  # Isaac ROS Visual SLAM (cuVSLAM)
  isaac_ros_visual_slam:
    type: git
    url: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
    version: main
  
  # Isaac ROS Nvblox (3D reconstruction)
  isaac_ros_nvblox:
    type: git
    url: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
    version: main
  
  # Isaac ROS Nitros (GPU 가속 통신)
  isaac_ros_nitros:
    type: git
    url: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
    version: main
  
  # ========== Sensors ==========
  # RealSense D435i 카메라
  realsense_ros:
    type: git
    url: https://github.com/IntelRealSense/realsense-ros.git
    version: ros2-development
  
  # Xsens MTi IMU 드라이버
  xsens_mti_driver:
    type: git
    url: https://github.com/DEMCON/ros2_xsens_mti_driver.git
    version: main
  
  # ========== Localization & Navigation ==========
  # Robot Localization (센서 퓨전: Wheel + IMU + VSLAM)
  robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: humble-devel
  
  # Navigation2
  navigation2:
    type: git
    url: https://github.com/ros-navigation/navigation2.git
    version: humble
EOF

# 통합 패키지 생성
cd src
ros2 pkg create --build-type ament_python scout_isaac \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros

# package.xml에 모든 의존성 작성
cat > scout_isaac/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>scout_isaac</name>
  <version>0.1.0</version>
  <description>Scout Mini Robot with Isaac ROS Integration</description>
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

  <!-- Scout 하드웨어 -->
  <depend>scout_msgs</depend>
  <depend>scout_base</depend>
  
  <!-- Isaac ROS Visual SLAM -->
  <exec_depend>isaac_ros_visual_slam</exec_depend>
  
  <!-- Isaac ROS Nvblox (3D Reconstruction) -->
  <exec_depend>isaac_ros_nvblox</exec_depend>
  
  <!-- RealSense 카메라 -->
  <exec_depend>ros-humble-realsense2-camera</exec_depend>
  <exec_depend>ros-humble-realsense2-description</exec_depend>
  
  <!-- IMU -->
  <depend>xsens_mti_driver</depend>
  
  <!-- Robot Localization (EKF 센서 퓨전) -->
  <exec_depend>ros-humble-robot-localization</exec_depend>
  
  <!-- Navigation2 -->
  <exec_depend>ros-humble-navigation2</exec_depend>
  <exec_depend>ros-humble-nav2-bringup</exec_depend>
  <exec_depend>ros-humble-nav2-map-server</exec_depend>
  
  <!-- URDF 및 로봇 모델 -->
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>
  
  <!-- 시각화 -->
  <exec_depend>ros-humble-rviz2</exec_depend>
  <exec_depend>ros-humble-rviz-visual-tools</exec_depend>
  
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
mkdir -p scout_isaac/launch
mkdir -p scout_isaac/config

# 통합 launch 파일 생성
cat > scout_isaac/launch/scout_isaac_bringup.launch.py <<'EOF'
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # ========== Hardware Layer ==========
        # Scout 베이스 컨트롤러
        Node(
            package='scout_base',
            executable='scout_base_node',
            name='scout_base',
            output='screen',
            parameters=[{
                'port_name': '/dev/ttyUSB0',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom'
            }]
        ),
        
        # RealSense D435i 카메라
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 'linear_interpolation',
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30'
            }]
        ),
        
        # Xsens MTi IMU
        Node(
            package='xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_imu',
            output='screen'
        ),
        
        # ========== Perception Layer ==========
        # Isaac ROS Visual SLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            output='screen',
            parameters=[{
                'enable_imu': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'enable_slam_visualization': True,
                'enable_observations_view': True,
                'enable_landmarks_view': True
            }],
            remappings=[
                ('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                ('stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
                ('visual_slam/imu', '/imu/data')
            ]
        ),
        
        # Isaac ROS Nvblox (3D Reconstruction)
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            name='nvblox_node',
            output='screen',
            parameters=[{
                'voxel_size': 0.05,
                'esdf_mode': '2d',
                'use_depth': True,
                'use_lidar': False,
                'integrate_depth_rate_hz': 40.0,
                'update_mesh_rate_hz': 5.0,
                'update_esdf_rate_hz': 10.0
            }],
            remappings=[
                ('depth/image', '/camera/depth/image_rect_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('color/image', '/camera/color/image_raw'),
                ('color/camera_info', '/camera/color/camera_info'),
                ('pose', '/visual_slam/tracking/odometry')
            ]
        ),
        
        # ========== Localization Layer ==========
        # Robot Localization (EKF: Wheel + IMU + VSLAM fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('scout_isaac'),
                'config', 'ekf.yaml'
            )],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),
        
        # ========== Navigation Layer ==========
        # Navigation2 (추후 별도 launch로 분리 권장)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(get_package_share_directory('nav2_bringup'),
        #                      'launch', 'navigation_launch.py')
        #     ]),
        #     launch_arguments={
        #         'use_sim_time': 'false',
        #         'params_file': os.path.join(
        #             get_package_share_directory('scout_isaac'),
        #             'config', 'nav2_params.yaml'
        #         )
        #     }.items()
        # ),
    ])
EOF

# EKF 설정 파일 생성
cat > scout_isaac/config/ekf.yaml <<'EOF'
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # Wheel Odometry (Scout)
    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_differential: false
    
    # IMU (Xsens MTi)
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    imu0_queue_size: 10
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    
    # Visual SLAM Odometry
    odom1: /visual_slam/tracking/odometry
    odom1_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_queue_size: 10
    odom1_differential: false
    
    # Process noise covariance
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]
