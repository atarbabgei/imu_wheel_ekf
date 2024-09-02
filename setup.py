from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_wheel_ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atar',
    maintainer_email='atarbabgei@gmail.com',
    description='Combine PX4 IMU and wheel odometry data to estimate pose',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = imu_wheel_ekf.ekf_node:main',
            'wheel_odometry_node = imu_wheel_ekf.wheel_odometry_node:main',
        ],
    },
)
