from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_wheel_ekf',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
        ),
        Node(
            package='imu_wheel_ekf',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
        ),
    ])
