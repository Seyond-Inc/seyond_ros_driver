import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('seyond')+'/rviz/rviz2.rviz'
    yaml_config=get_package_share_directory('seyond')+'/config.yaml'

    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value=yaml_config,
        description='Path to the configuration file'
    )

    container = ComposableNodeContainer(
            name='seyond_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='seyond',
                    plugin='seyond::SeyondDriver',
                    name='seyond_node',
                    parameters=[
                        {'config_path': LaunchConfiguration('config_path')},
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
    )

    return launch.LaunchDescription([
        declare_config_path,
        container,
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
        ])
