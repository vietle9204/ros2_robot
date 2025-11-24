from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robot_2wd_urdf').find('robot_2wd_urdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_2wd.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            )
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'robot_2wd'],
            output='screen'
        )
    ])

