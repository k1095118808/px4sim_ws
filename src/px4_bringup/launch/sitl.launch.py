from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch Arguments
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.environ.get('PX4_DIR', '/home/kxd/ws/px4sim_ws/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )
    
    enable_px4_arg = DeclareLaunchArgument(
        'enable_px4', default_value='true', description='Enable PX4 SITL'
    )
    enable_agent_arg = DeclareLaunchArgument(
        'enable_agent', default_value='true', description='Enable MicroXRCEAgent'
    )
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge', default_value='true', description='Enable ROS-GZ Bridge'
    )

    # MicroXRCEAgent process
    # Connects PX4 to ROS 2 via DDS
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_agent'))
    )

    # PX4 SITL process
    # Runs 'make px4_sitl gz_x500_depth' with baylands world
    px4_env = os.environ.copy()
    px4_env['PX4_GZ_WORLD'] = 'baylands'
    
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        env=px4_env,
        output='screen',
        shell=True,
        condition=IfCondition(LaunchConfiguration('enable_px4'))
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/camera/image_raw'),
            ('/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', '/camera/camera_info'),
            ('/depth_camera', '/camera/depth/image_raw')
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge'))
    )

    return LaunchDescription([
        px4_dir_arg,
        enable_px4_arg,
        enable_agent_arg,
        enable_bridge_arg,
        px4_sitl,
        micro_xrce_agent,
        bridge
    ])
