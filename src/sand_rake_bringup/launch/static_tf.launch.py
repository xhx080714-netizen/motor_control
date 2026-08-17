from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------------------------------------------------
    # Enable switches
    # ---------------------------------------------------------
    enable_front_camera_tf = LaunchConfiguration(
        'enable_front_camera_tf'
    )

    enable_rear_camera_tf = LaunchConfiguration(
        'enable_rear_camera_tf'
    )

    # ---------------------------------------------------------
    # Front camera pose
    #
    # All values remain TBD until the real installation pose
    # is measured and confirmed.
    # ---------------------------------------------------------
    front_x = LaunchConfiguration('front_x')
    front_y = LaunchConfiguration('front_y')
    front_z = LaunchConfiguration('front_z')

    front_roll = LaunchConfiguration('front_roll')
    front_pitch = LaunchConfiguration('front_pitch')
    front_yaw = LaunchConfiguration('front_yaw')

    # ---------------------------------------------------------
    # Rear camera pose
    # ---------------------------------------------------------
    rear_x = LaunchConfiguration('rear_x')
    rear_y = LaunchConfiguration('rear_y')
    rear_z = LaunchConfiguration('rear_z')

    rear_roll = LaunchConfiguration('rear_roll')
    rear_pitch = LaunchConfiguration('rear_pitch')
    rear_yaw = LaunchConfiguration('rear_yaw')

    # ---------------------------------------------------------
    # Front camera static TF
    # ---------------------------------------------------------
    front_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_camera_static_tf_publisher',
        output='screen',
        arguments=[
            '--x', front_x,
            '--y', front_y,
            '--z', front_z,
            '--roll', front_roll,
            '--pitch', front_pitch,
            '--yaw', front_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_camera_optical_frame',
        ],
        condition=IfCondition(enable_front_camera_tf),
    )

    # ---------------------------------------------------------
    # Rear camera static TF
    # ---------------------------------------------------------
    rear_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_camera_static_tf_publisher',
        output='screen',
        arguments=[
            '--x', rear_x,
            '--y', rear_y,
            '--z', rear_z,
            '--roll', rear_roll,
            '--pitch', rear_pitch,
            '--yaw', rear_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'rear_camera_optical_frame',
        ],
        condition=IfCondition(enable_rear_camera_tf),
    )

    # ---------------------------------------------------------
    # BLOCKED information
    # ---------------------------------------------------------
    front_camera_blocked_log = LogInfo(
        condition=UnlessCondition(enable_front_camera_tf),
        msg=(
            '[TF] front camera static TF is disabled: '
            'installation pose is still TBD.'
        ),
    )

    rear_camera_blocked_log = LogInfo(
        condition=UnlessCondition(enable_rear_camera_tf),
        msg=(
            '[TF] rear camera static TF is disabled: '
            'installation pose is still TBD.'
        ),
    )

    lidar_blocked_log = LogInfo(
        msg=(
            '[TF] LiDAR static TF is BLOCKED/deferred: '
            'LiDAR integration is currently paused.'
        ),
    )

    return LaunchDescription([
        # Enable arguments
        DeclareLaunchArgument(
            'enable_front_camera_tf',
            default_value='false',
            description='Enable front camera static TF publisher',
        ),

        DeclareLaunchArgument(
            'enable_rear_camera_tf',
            default_value='false',
            description='Enable rear camera static TF publisher',
        ),

        # Front camera pose
        DeclareLaunchArgument(
            'front_x',
            default_value='TBD',
            description='Front camera x relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'front_y',
            default_value='TBD',
            description='Front camera y relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'front_z',
            default_value='TBD',
            description='Front camera z relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'front_roll',
            default_value='TBD',
            description='Front camera roll relative to base_link [rad]',
        ),

        DeclareLaunchArgument(
            'front_pitch',
            default_value='TBD',
            description='Front camera pitch relative to base_link [rad]',
        ),

        DeclareLaunchArgument(
            'front_yaw',
            default_value='TBD',
            description='Front camera yaw relative to base_link [rad]',
        ),

        # Rear camera pose
        DeclareLaunchArgument(
            'rear_x',
            default_value='TBD',
            description='Rear camera x relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'rear_y',
            default_value='TBD',
            description='Rear camera y relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'rear_z',
            default_value='TBD',
            description='Rear camera z relative to base_link [m]',
        ),

        DeclareLaunchArgument(
            'rear_roll',
            default_value='TBD',
            description='Rear camera roll relative to base_link [rad]',
        ),

        DeclareLaunchArgument(
            'rear_pitch',
            default_value='TBD',
            description='Rear camera pitch relative to base_link [rad]',
        ),

        DeclareLaunchArgument(
            'rear_yaw',
            default_value='TBD',
            description='Rear camera yaw relative to base_link [rad]',
        ),

        # Logs
        front_camera_blocked_log,
        rear_camera_blocked_log,
        lidar_blocked_log,

        # TF publishers
        front_camera_tf_node,
        rear_camera_tf_node,
    ])
