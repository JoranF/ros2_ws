from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path, get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_path('smart_car')
    world_file = os.path.join(pkg_share, 'worlds', 'smalltown.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'smart_car.urdf') # Note: Best practice is to use the xacro file, but urdf is fine.

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])

    return LaunchDescription([
        # Add this parameter to the Gazebo launch itself
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'smart_car'],
            output='screen'
        ),

        # Publish the URDF to /robot_description for RViz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

    ])