from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vehicle_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ydiwan',
    maintainer_email='youssefdiwan08@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_loader_node = vehicle_controller_py.waypoint_loader_node:main',
            'vehicle_controller_node = vehicle_controller_py.vehicle_controller_node:main',
            'wp_follower = vehicle_controller_py.wp_follower:main',
            'teleop_bridge = vehicle_controller_py.teleop_bridge:main',
        ],
    },
)
