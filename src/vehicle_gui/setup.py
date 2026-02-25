import os
from setuptools import setup
from glob import glob

package_name = 'vehicle_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include map images
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='car',
    maintainer_email='car@example.com',
    description='Waypoint selection GUI for cyber city vehicle',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_gui = vehicle_gui.waypoint_gui:main',
        ],
    },
)