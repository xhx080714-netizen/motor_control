import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_test_event = LaunchConfiguration('enable_test_event')
    control_launch = os.path.join(
        get_package_share_directory('sand_rake_control'),
        'launch',
        'chassis_mock.launch.xml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_test_event',
            default_value='false',
            description='Enable the team SafetyEvent test publisher',
        ),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(control_launch)),
        Node(
            package='sand_rake_interfaces',
            executable='safety_event_py_publisher',
            output='screen',
            condition=IfCondition(enable_test_event),
        ),
    ])
