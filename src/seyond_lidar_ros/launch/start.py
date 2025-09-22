from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rviz_config=get_package_share_directory('seyond')+'/rviz/rviz2.rviz'

    return LaunchDescription(
        [
            # set log color
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),

            DeclareLaunchArgument(
                'config_path',
                default_value='',
                description='config path'
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='limit log from lidar'
            ),
            DeclareLaunchArgument(
                'replay_rosbag',
                default_value='False',
                description='replay rosbag flag'
            ),
            DeclareLaunchArgument(
                'packet_mode',
                default_value='False',
                description='packets topic enable/disable'
            ),
            DeclareLaunchArgument(
                'aggregate_num',
                default_value='20',
                description='aggregate packets num'
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='seyond',
                description='ros frame id'
            ),
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
                'lidar_name',
                default_value='seyond',
                description='lidar name'
            ),
            DeclareLaunchArgument(
                'lidar_ip',
                default_value='172.168.1.10',
                description='lidar ip'
            ),
            DeclareLaunchArgument(
                'port',
                default_value='8010',
                description='recv data from tcp port(like 8010)'
            ),
            DeclareLaunchArgument(
                'udp_port',
                default_value='8010',
                description='recv data from udp port(like 8010), or use tcp as default(-1)'
            ),
            DeclareLaunchArgument(
                'reflectance_mode',
                default_value='True',
                description='set lidar point cloud reflect mode, True or False'
            ),
            DeclareLaunchArgument(
                'multiple_return',
                default_value='1',
                description='set lidar point cloud multiple return mode, 1/2/3'
            ),
            DeclareLaunchArgument(
                'enable_falcon_ring',
                default_value='False',
                description='enable falcon ring_id calculation, default false'
            ),

            DeclareLaunchArgument(
                'continue_live',
                default_value='False',
                description='Continue live'
            ),

            DeclareLaunchArgument(
                'pcap_file',
                default_value='',
                description='Path to pcap file'
            ),
            DeclareLaunchArgument(
                'hv_table_file',
                default_value='',
                description='Path to hv table file'
            ),
            DeclareLaunchArgument(
                'packet_rate',
                default_value='10000',
                description='play packet rate, default is 20, 0 means play as fast as possible, e.g, 15000 means play at 1.5x speed'
            ),
            DeclareLaunchArgument(
                'file_rewind',
                default_value='0',
                description='file rewind, 0 means no rewind, < 0 means rewind infinity times'
            ),
            DeclareLaunchArgument(
                'max_range',
                default_value='2000.0',
                description='ros point cloud max range'
            ),
            DeclareLaunchArgument(
                'min_range',
                default_value='0.4',
                description='ros point cloud min range'
            ),
            DeclareLaunchArgument(
                'name_value_pairs',
                default_value='',
                description='name value pairs, e.g., "name1=value1,name2=value2"'
            ),
            DeclareLaunchArgument(
                'coordinate_mode',
                default_value='3',
                description='specify coordinate transformation, default is 3, (x/y/z) 0:up/right/forward 3:forward/left/up'
            ),
            DeclareLaunchArgument(
                'transform_enable',
                default_value='false',
                description='transform enable'
            ),
            DeclareLaunchArgument(
                'x',
                default_value='0.0',
                description='x'
            ),
            DeclareLaunchArgument(
                'y',
                default_value='0.0',
                description='y'
            ),
            DeclareLaunchArgument(
                'z',
                default_value='0.0',
                description='z'
            ),
            DeclareLaunchArgument(
                'pitch',
                default_value='0.0',
                description='pitch'
            ),
            DeclareLaunchArgument(
                'yaw',
                default_value='0.0',
                description='yaw'
            ),
            DeclareLaunchArgument(
                'roll',
                default_value='0.0',
                description='roll'
            ),
            DeclareLaunchArgument(
                'transform_matrix',
                default_value='',
                description='transform matrix string, if not empty, priority is higher than x/y/z/pitch/yaw/roll'
            ),
            

            Node(
                package="seyond",
                executable="seyond_node",
                parameters=[
                    {'config_path': LaunchConfiguration('config_path')},
                    {'log_level': LaunchConfiguration('log_level')},
                    {'replay_rosbag': LaunchConfiguration('replay_rosbag')},
                    {'packet_mode': LaunchConfiguration('packet_mode')},
                    {'aggregate_num': LaunchConfiguration('aggregate_num')},
                    {'frame_id': LaunchConfiguration('frame_id')},
                    {'frame_topic': LaunchConfiguration('frame_topic')},
                    {'packet_topic': LaunchConfiguration('packet_topic')},
                    {'lidar_name': LaunchConfiguration('lidar_name')},
                    {'lidar_ip': LaunchConfiguration('lidar_ip')},
                    {'port': LaunchConfiguration('port')},
                    {'udp_port': LaunchConfiguration('udp_port')},
                    {'reflectance_mode': LaunchConfiguration('reflectance_mode')},
                    {'multiple_return': LaunchConfiguration('multiple_return')},
                    {'enable_falcon_ring': LaunchConfiguration('enable_falcon_ring')},
                    {'continue_live': LaunchConfiguration('continue_live')},
                    {'pcap_file': LaunchConfiguration('pcap_file')},
                    {'hv_table_file': LaunchConfiguration('hv_table_file')},
                    {'packet_rate': LaunchConfiguration('packet_rate')},
                    {'file_rewind': LaunchConfiguration('file_rewind')},
                    {'max_range': LaunchConfiguration('max_range')},
                    {'min_range': LaunchConfiguration('min_range')},
                    {'name_value_pairs': LaunchConfiguration('name_value_pairs')},
                    {'coordinate_mode': LaunchConfiguration('coordinate_mode')},
                    {'transform_enable': LaunchConfiguration('transform_enable')},
                    {'x': LaunchConfiguration('x')},
                    {'y': LaunchConfiguration('y')},
                    {'z': LaunchConfiguration('z')},
                    {'pitch': LaunchConfiguration('pitch')},
                    {'yaw': LaunchConfiguration('yaw')},
                    {'roll': LaunchConfiguration('roll')},
                    {'transform_matrix': LaunchConfiguration('transform_matrix')},
                ],
            ),
            Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
        ]
    )
