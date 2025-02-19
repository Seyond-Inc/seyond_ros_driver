from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable

import os


def generate_launch_description():
    return LaunchDescription(
        [
            # set log color
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),

            DeclareLaunchArgument(
                'frame_topic',
                default_value='iv_points',
                description='output topic name'
            ),
            DeclareLaunchArgument(
                'packet_topic',
                default_value='iv_packets',
                description='output topic name'
            ),
            DeclareLaunchArgument(
                'stamp',
                default_value='False',
                description='print stamp or not'
            ),
            DeclareLaunchArgument(
                'hz',
                default_value='False',
                description='print hz or not'
            ),
            DeclareLaunchArgument(
                'window',
                default_value='30',
                description='test window'
            ),
            DeclareLaunchArgument(
                'packet_size',
                default_value='False',
                description='print packet_size per frame'
            ),
            DeclareLaunchArgument(
                'packet_loss_rate',
                default_value='False',
                description='print packet loss rate'
            ),

            Node(
                package="seyond",
                executable="seyond_test",
                parameters=[
                    {'frame_topic': LaunchConfiguration('frame_topic')},
                    {'packet_topic': LaunchConfiguration('packet_topic')},
                    {'stamp': LaunchConfiguration('stamp')},
                    {'hz': LaunchConfiguration('hz')},
                    {'window': LaunchConfiguration('window')},
                    {'packet_size': LaunchConfiguration('packet_size')},
                    {'packet_loss_rate': LaunchConfiguration('packet_loss_rate')},
                ],
            ),
        ]
    )
