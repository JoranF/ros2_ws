#!/usr/bin/env python3
"""
Launch file for the smart car localization system.
This launch file includes:
- Joint state publisher (publishes joint states from vehicle status)
- Robot localization EKF node (fuses odometry and IMU data)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('smart_car')
    
    # Path to the EKF configuration file
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        # Joint State Publisher Node
        # Estimates robot wheel 3D location using steering_angle_rad and engine_speed_rpm
        # Publishes joint transformations using TF broadcaster
        Node(
            package='smart_car',
            executable='joint_state_publisher.py',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'wheel_diameter': 0.064},
                {'wheelbase_length': 0.257},
                {'publish_rate': 50.0}
            ]
        ),
        
        # Robot Localization EKF Node
        # Fuses wheel odometry (/smart_car/odom) and IMU data (/imu_data)
        # Publishes fused pose estimate to /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('odometry/filtered', '/odom')
            ]
        ),
    ])
