import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name
    pkg_name = 'trash_boy'
    
    # Paths
    pkg_share = FindPackageShare(pkg_name)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'model.urdf'])
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'trash_boy',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # Color tracker node
    color_tracker = Node(
        package=pkg_name,
        executable='object_tracker',
        name='color_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': False  # Uses real camera, not sim time
        }]
    )
    
    # Object follower node
    object_follower = Node(
        package=pkg_name,
        executable='object_follower',
        name='object_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'kp_x': 0.005,  # Increased for faster response
            'kp_y': 0.005,  # Increased for faster response
            'kd_x': 0.001,  # Damping to reduce oscillation
            'kd_y': 0.001,  # Damping to reduce oscillation
            'max_linear_speed': 1.0,  # Increased max speed
            'min_speed_threshold': 0.05,  # Minimum speed threshold
            'deadzone': 30.0,  # Slightly larger deadzone
            'enable_following': True
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        color_tracker,
        object_follower
    ])