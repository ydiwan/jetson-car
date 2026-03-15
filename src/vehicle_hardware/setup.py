from setuptools import find_packages, setup

package_name = 'vehicle_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'maestro_interface_node = vehicle_hardware.maestro_interface_node:main',
            'ackermann_kinematics_node = vehicle_hardware.ackermann_kinematics_node:main',
            'gpio_node = vehicle_hardware.gpio_node:main',
        ],
    },
)
