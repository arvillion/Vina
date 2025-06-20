from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
     
    return LaunchDescription([
        # Launch the woosh agent node
        Node(
            package='woosh_robot_agent',
            executable='agent',
            namespace='woosh_robot',
            parameters=[{'ip': '169.254.128.2'}], 
            output='screen',
            respawn=True,  # 自动重启
            respawn_delay=1.0,  # 可选：重启前等待秒数
        ),
        # Redirect the twist message from /cmd_vel to the woosh service.
        Node(
            package='twist_redirector',
            executable='run',
            output='log'
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
            parameters=[]
        ),
        # publish dynamic odom->base_footprint transform based on /odom topic
        Node(
            package='adapter',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='log'
        ),
        # publish static base_footprint->laser_link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=[
                '0.2455', '0', '0.213',      # x, y, z（laser_link 相对 base_footprint 的位置）
                '0', '0', '0',        # roll, pitch, yaw（laser_link 相对 base_footprint 的姿态，单位是弧度）
                'base_footprint',     # 父坐标系
                'laser_link'          # 子坐标系
            ]
        )
        # Redirect the pose message from /initialpose to the woosh service.
        # Node(
        #     package='adapter',
        #     executable='initialpose',
        #     output='log'
        # ),
    ])