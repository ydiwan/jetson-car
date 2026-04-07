from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'odometry_estimator_node = sensor_fusion.odometry_estimator_node:main',
            'vicon_converter_node = sensor_fusion.vicon_converter_node:main',
            'sim_ground_truth_node = sensor_fusion.sim_ground_truth_node:main',
            'gpio_node = sensor_fusion.gpio_node:main',
            'joy_node = sensor_fusion.joy_node:main'
        ],
    },
)
