from setuptools import setup
from glob import glob
import os

package_name = 'map_contruct'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicky',
    maintainer_email='your.email@example.com',
    description='Package for converting point clouds to 2D laser scans and performing SLAM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag_to_slam = map_contruct.rosbag_to_slam:main',
            'cost_layer_visualizer = map_contruct.scripts.cost_layer_visualizer:main',
            'cost_layer_processor = map_contruct.scripts.cost_layer_processor:main',
            'test_marker = map_contruct.scripts.test_marker:main',
        ],
    },
)
