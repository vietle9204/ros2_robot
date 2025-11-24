from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    yaml_file = '/home/vietle9204/ros2_2wd_ws/src/robot_2wd_control/config/robot_2wd_control.yaml'

    # Node chạy controller_manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[yaml_file],
        output="both",
    )

    # Spawner cho joint_state_broadcaster
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner cho diff_drive_controller (hoặc controller khác)
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Đăng ký sự kiện: khi ros2_control_node start thì spawn controller
    event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_spawner, diff_drive_spawner],
        )
    )

    return LaunchDescription([
        ros2_control_node,
        event_handler
    ])

