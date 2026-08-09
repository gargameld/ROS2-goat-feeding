from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_dir={},
    install_requires=[
        'setuptools',
        'rclpy',
        'tf2_ros',
        'geometry_msgs',
        'xacro',
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF/XACRO files: arm + gripper
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'gripper'), glob('urdf/gripper/*.xacro')),
        (os.path.join('share', package_name, 'urdf/gripper/meshes'), glob('urdf/gripper/meshes/*')),
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gripper_debugger = robot_description.gripper_debugger:main'
        ],
    },
)
