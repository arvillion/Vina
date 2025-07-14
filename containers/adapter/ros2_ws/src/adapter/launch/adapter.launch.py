from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition




def generate_launch_description():

    just_woosh_agent = LaunchConfiguration('just_woosh_agent')
    
    desc = LaunchDescription()
    
    desc.add_action(DeclareLaunchArgument(
        'just_woosh_agent',
        default_value='false',
        description='If true, only launch the woosh agent.'
    ))
    
    # woosh-robot-agent node
    desc.add_action(Node(
        package='woosh_robot_agent',
        executable='agent',
        namespace='woosh_robot',
        parameters=[{'ip': '169.254.128.2'}], 
        output='screen',
        respawn=True,  # 自动重启
        respawn_delay=1.0,  # 重启前等待秒数
    ))
    
    # Redirect the twist message from /cmd_vel to the woosh service.
    desc.add_action(Node(
        package='twist_redirector',
        executable='run',
        output='log',
        condition=UnlessCondition(just_woosh_agent),
    ))
    
    desc.add_action(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='log',
        parameters=[],
        condition=UnlessCondition(just_woosh_agent),
    ))
        
    # publish dynamic odom->base_footprint transform based on /odom topic
    desc.add_action(Node(
        package='adapter',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='log',
        condition=UnlessCondition(just_woosh_agent),
    ))
        
    # publish static base_footprint->laser_link transform
    desc.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.2455', '0', '0.213',      # x, y, z（laser_link 相对 base_footprint 的位置）
            '0', '0', '0',        # roll, pitch, yaw（laser_link 相对 base_footprint 的姿态，单位是弧度）
            'base_footprint',     # 父坐标系
            'laser_link'          # 子坐标系
        ],
        condition=UnlessCondition(just_woosh_agent),
    ))
    
    # Redirect the pose message from /initialpose to the woosh service.
    # Node(
    #     package='adapter',
    #     executable='initialpose',
    #     output='log'
    # ),
    return desc