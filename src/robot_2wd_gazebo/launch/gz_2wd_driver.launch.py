from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # --- Controller spawner nodes ---
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- Teleop keyboard ---
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        arguments=[
            '--ros-args',
            '-r', '/cmd_vel:=/diff_drive_controller/cmd_vel',
            '-p', 'stamped:=True'
        ]
    )
    
    # Delay 2s để controller_manager do plugin ros2_control tạo xong
    controller_spawner_timer = TimerAction(
        period=2.0,
        actions=[teleop_node]
    )

    return LaunchDescription([
        joint_state_spawner,
        diff_drive_spawner,
        controller_spawner_timer,
    ])

