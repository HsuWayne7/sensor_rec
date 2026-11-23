import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('odom_frame', default_value='t265_odom'),
        DeclareLaunchArgument('base_frame', default_value='t265_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('enable_fisheye', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument(
            'pyrealsense_path',
            default_value=os.path.expanduser('~/librealsense/build/wrappers/python')),

        # t265_link -> 機器人 base_link(選用)
        DeclareLaunchArgument('publish_base_link', default_value='false'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('bx', default_value='0.0'),
        DeclareLaunchArgument('by', default_value='0.0'),
        DeclareLaunchArgument('bz', default_value='0.0'),
        DeclareLaunchArgument('byaw', default_value='0.0'),
    ]

    t265 = Node(
        package='t265_ros',
        executable='t265_node',
        name='t265_node',
        output='screen',
        parameters=[{
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'enable_fisheye': LaunchConfiguration('enable_fisheye'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'pyrealsense_path': LaunchConfiguration('pyrealsense_path'),
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        condition=IfCondition(LaunchConfiguration('publish_base_link')),
        arguments=[
            '--x', LaunchConfiguration('bx'),
            '--y', LaunchConfiguration('by'),
            '--z', LaunchConfiguration('bz'),
            '--yaw', LaunchConfiguration('byaw'),
            '--pitch', '0', '--roll', '0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('robot_base_frame'),
        ],
    )

    return LaunchDescription(args + [t265, static_tf])
