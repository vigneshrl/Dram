from setuptools import setup
from glob import glob
import os

package_name = 'map_contruct'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicky',
    maintainer_email='your.email@example.com',
    description='ROS2 package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'inference = map_contruct.inference:main',
            'pointcloud_segmenter = map_contruct.pointcloud_segmenter:main',
            'recovery_points = map_contruct.recovery_points:main',
            'cost_layer_processor = map_contruct.cost_layer_processor:main',
            'odom_tf_broadcaster = map_contruct.odom_tf_brodcaster:main',
        ], 
    },
)
