import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'grasp_pose_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'open3d', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='gargameld',
    maintainer_email='yotam.ambar@gmail.com',
    description='Estimates a grasp pose by ICP-registering a stored model point '
                'cloud against a captured camera point cloud.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'grasp_pose_provider_node = grasp_pose_provider.main:main',
        ],
    },
)
