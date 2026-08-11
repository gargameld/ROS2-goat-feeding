from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Project the front depth stream to an XYZ PointCloud2 message."""
    return LaunchDescription([
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='front_depth_pointcloud',
            remappings=[
                ('image_rect', '/front_camera/depth/image_raw'),
                ('camera_info', '/front_camera/camera_info'),
                ('points', '/front/depth/points'),
            ],
        ),
    ])
