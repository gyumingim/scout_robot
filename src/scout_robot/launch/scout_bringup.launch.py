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
