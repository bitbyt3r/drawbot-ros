from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="drawbot").find("drawbot")

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                { "robot_description": Command(['xacro ', os.path.join(pkg_share, "description/drawbot.urdf")])}
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='drawbot',
            executable='motors',
            name='motors',
            parameters=[
                {"base_frame": "base_link"},
                {"topic": "drawbot/encoders/odom"}
            ]
        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055_imu_node',
            parameters=[
                { "ros_topic_prefix": "drawbot/" },
                { "connection_type": "i2c" },
                { "i2c_bus": 1 },
                { "i2c_addr": 0x28 },
                { "imu_query_frequency": 100 },
                { "rot_query_frequency": 0 },
                { "gra_query_frequency": 0 },
                { "mag_query_frequency": 0 },
                { "tmp_query_frequency": 1 },
                { "cal_query_frequency": 1 },
                { "frame_id": "imu_link" },
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
            package='drawbot',
            executable='uwb',
            name='right_uwb',
            parameters=[
                {"port_name": "/dev/serial/by-id/usb-SEGGER_J-Link_000760190046-if00"},
                {"refresh_interval": 0.1},
                {"frame": "map"},
                {"topic": "drawbot/uwb/right"},
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                {"frequency": 10.0},
                {"sensor_timeout": 0.2},
                {"two_d_mode": True},
                {"publish_tf": True},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"world_frame": "odom"},
                {"odom0": "drawbot/encoders/odom"},
                {"odom0_config": [
                    False, False, False,
                    False, False, False,
                    True, True, True,
                    False, False, True,
                    False, False, False
                ]},
                {"imu0": "drawbot/imu"},
                {"imu0_config": [
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    True, True, True,
                ]}
            ],
            remappings=[
                ("odometry/filtered", "odometry/filtered_twist")
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                {"frequency": 10.0},
                {"sensor_timeout": 0.2},
                {"two_d_mode": True},
                {"publish_tf": True},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"world_frame": "map"},
                {"odom0": "drawbot/encoders/odom"},
                {"odom0_config": [
                    False, False, False,
                    False, False, False,
                    True, True, True,
                    False, False, True,
                    False, False, False
                ]},
                {"odom0_pose_rejection_threshold": 0.2},
                {"odom0_twist_rejection_threshold": 0.2},
                {"imu0": "drawbot/imu"},
                {"imu0_config": [
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    True, True, True,
                ]},
                {"imu0_pose_rejection_threshold": 0.1},
                {"imu0_twist_rejection_threshold": 0.2},
                {"pose0": "drawbot/uwb/right"},
                {"pose0_config": [
                    True, True, True,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                ]},
                {"pose0_pose_rejection_threshold": 0.5},
                {"pose0_twist_rejection_threshold": 0.2},
            ],
            remappings=[
                ("odometry/filtered", "odometry/filtered_map")
            ]
        )
    ])