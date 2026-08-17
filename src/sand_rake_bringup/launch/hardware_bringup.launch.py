from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_chassis = LaunchConfiguration('enable_chassis')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lidar = LaunchConfiguration('enable_lidar')

    safety_controller = Node(
        package='sand_rake_control',
        executable='safety_controller',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_chassis', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_lidar', default_value='false'),
        safety_controller,
        LogInfo(
            condition=IfCondition(enable_chassis),
            msg='[BLOCKED] The production chassis driver is not integrated.',
        ),
        LogInfo(
            condition=IfCondition(enable_camera),
            msg='[BLOCKED] The camera adapter is not integrated.',
        ),
        LogInfo(
            condition=IfCondition(enable_lidar),
            msg='[BLOCKED] LiDAR integration is deferred.',
        ),
    ])
