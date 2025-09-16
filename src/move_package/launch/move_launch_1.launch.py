from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for optional components
        DeclareLaunchArgument(
            'use_slam_toolbox',
            default_value='false',
            description='Whether to launch SLAM toolbox'
        ),

        # Core system node - Sistema Operativo (main controller)
        Node(
            package='move_package',
            executable='sistemaoperativo',
            name='sistema_operativo',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # Obstacle detection and avoidance node
        Node(
            package='navigation_node',
            executable='nav',
            name='obstacle_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # Mapping node for exploration
        Node(
            package='mapping_node',
            executable='mapping_node',
            name='mapping_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            remappings=[
                ('/vel_mapping', '/mapping_vel')
            ]
        ),

        # Come back home node
        Node(
            package='come_back_home',
            executable='come_back_home',
            name='home_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # Velocity modifier for navigation
        Node(
            package='vel_modifier',
            executable='velocity_modifier',
            name='velocity_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # SLAM Toolbox (optional, controlled by launch argument)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'slam_toolbox': {
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'scan_subscriber_queue_size': 5,
                    'scan_publisher_queue_size': 5,
                    'map_update_interval': 5.0,
                }},
            ],
            condition=IfCondition(LaunchConfiguration('use_slam_toolbox'))
        ),

        # Optional: Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '$(find-pkg-share slam_toolbox)/config/mapper_params_online_async.rviz'],
            parameters=[
                {'use_sim_time': True}
            ],
            condition=IfCondition(LaunchConfiguration('use_slam_toolbox'))
        ),

        # Status monitor node (optional utility)
        Node(
            package='rqt_topic',
            executable='rqt_topic',
            name='status_monitor',
            output='screen',
            condition=IfCondition('false')  # Set to 'true' to enable
        ),

        # Delayed start for some nodes to ensure proper initialization
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_to_laser',
                    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
                    parameters=[
                        {'use_sim_time': True}
                    ]
                )
            ]
        ),
    ])