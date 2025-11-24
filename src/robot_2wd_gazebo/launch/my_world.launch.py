from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('robot_2wd_gazebo')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'my_world.sdf'])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),
    ])

