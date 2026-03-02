from setuptools import find_packages, setup

package_name = 'video_nodes_py'

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
            'video_publisher = video_nodes_py.video_publisher_node:main',
            'video_subscriber = video_nodes_py.video_subscriber_node:main',
            'video_publisher_test = video_nodes_py.video_publisher_node_test:main',
            'traffic_light = video_nodes_py.traffic_light_node:main',
        ],
    },
)
