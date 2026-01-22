import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Static TF: Fake the wheels (Odom -> Base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # 2. Static TF: Position the Lidar (Base -> Laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # 3. LiDAR Driver
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 460800, 
                         'frame_id': 'laser',
                         'inverted': False, 
                         'angle_compensate': True}],
            output='screen'
        ),

        # 4. SLAM Toolbox (Now forced to update continuously)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
               {'use_sim_time': False},
               {'odom_frame': 'odom'},
               {'base_frame': 'base_link'},
               {'map_frame': 'map'},
               {'scan_topic': '/scan'},
               # FORCE MAP UPDATES even if odom is 0
               {'minimum_travel_distance': 0.0},
               {'minimum_travel_heading': 0.0},
               {'map_update_interval': 1.0}
            ]
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])    
