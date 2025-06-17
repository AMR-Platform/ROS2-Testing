from setuptools import setup

package_name = 'serial_to_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package to read Arduino serial and publish IMU and Odometry topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_to_ros2_node = serial_to_ros2.serial_to_ros2_node:main',
        ],
    },
)
