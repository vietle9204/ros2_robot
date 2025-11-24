from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    yaml_file = '/home/vietle9204/ros2_2wd_ws/src/robot_2wd_control/config/robot_2wd_control.yaml'
    urdf_pkg_path = FindPackageShare('model')
    default_model_path = PathJoinSubstitution([urdf_pkg_path, 'urdf', 'robot_2wd.xacro'])
    default_rviz_config_path = PathJoinSubstitution([urdf_pkg_path, 'rviz', 'urdf.rviz'])
    
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                     description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')   
    model_arg = DeclareLaunchArgument(name='robot_model', default_value=default_model_path,
                                     description='Path to robot urdf file relative to model package')
                                     
    #robot description                                  
    robot_desc = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'model',
            'urdf_package_path': LaunchConfiguration('robot_model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    )                                 

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
        gui_arg,
        rviz_arg,
        model_arg,
        robot_desc,
        ros2_control_node,
        event_handler,
    ])

