from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # ArUco Detector
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[{
                'marker_size': 0.15,
                'camera_frame': 'camera_color_optical_frame',
                'reference_frame': 'map'  # Added for TF consistency
            }]
        ),
        
        # Static TF from camera to base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        
        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': get_package_share_directory('my_robot_launch') + '/config/nav2_params.yaml'
            }.items()
        )
    ])
