from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drawbot',
            executable='motors',
            name='motors'
        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055_imu_node',
            parameters=[
                { "ros_topic_prefix": "imu/" },
                { "connection_type": "i2c" },
                { "i2c_bus": 1 },
                { "i2c_addr": 0x28 },
                { "imu_query_frequency": 100 },
                { "rot_query_frequency": 0 },
                { "gra_query_frequency": 0 },
                { "mag_query_frequency": 0 },
                { "tmp_query_frequency": 1 },
                { "cal_query_frequency": 1 },
                { "frame_id": "base_footprint" },
                { "operation_mode": 0x0C },
                { "placement_axis_remap": "P6" },
                { "acc_factor": 100.0 },
                { "mag_factor": 16000000.0 },
                { "gyr_factor": 900.0 },
                { "set_offsets" : False }
            ],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=["/home/pi/ros2_ws/config/ekf.yaml"]
        )
    ])