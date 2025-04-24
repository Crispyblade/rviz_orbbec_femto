from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='point_cloud_xyzrgb_node',
            remappings=[
                ('rgb/image_rect_color', '/camera/color/image_raw'),
                ('depth_registered/image_rect', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('points', '/camera/depth/color_points')
            ]
        ),  # <-- COMMA ADDED HERE

    ])
