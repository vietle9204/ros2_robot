from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- Paths ---
    pkg_gazebo = FindPackageShare('robot_2wd_gazebo')
    pkg_model = FindPackageShare('model')
    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', 'my_world.sdf'])
#    robot_xacro = PathJoinSubstitution([pkg_model, 'urdf', 'gz_robot_2wd.xacro'])

    # --- Launch Gazebo ---
    gz_process = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    # --- Robot description publisher ---
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_model, 'launch', 'robot_desc.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_model': 'gz_robot_2wd.xacro'
        }.items()
    )
#    robot_state_publisher_node = Node(
#        package='robot_state_publisher',
#        executable='robot_state_publisher',
#        name='robot_state_publisher',
#        output='screen',
#        parameters=[{
#            'use_sim_time': True,
#            'robot_description': Command(['xacro ', robot_xacro])
#        }]
#    )

    # --- Spawn robot into Gazebo ---
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': True, 'publish_clock': True}],
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_2wd',
            '-allow_renaming', 'true',
            '-z', '0.1'
        ],
    )
    
    # Bridge cho msg từ Gazebo sang ROS 2
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',

            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen',
        remappings=[('/clock', '/clock')]
    )
    

    return LaunchDescription([
        gz_process,
        robot_desc_launch,
        spawn_robot_node,
        node_gz_bridge,
    ])

