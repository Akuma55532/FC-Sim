"""Start ground-station MAVROS with upstream PX4 defaults and a GCS heartbeat."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mavros = Path(get_package_share_directory('mavros')) / 'launch'
    core = Path(get_package_share_directory('flight_core'))
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14580',
                              description='Flight-controller connection URL (defaults to local PX4 SITL)'),
        Node(
            package='mavros', executable='mavros_node', output='screen',
            parameters=[
                str(mavros / 'px4_pluginlists.yaml'),
                str(mavros / 'px4_config.yaml'),
                # Load last so only the intended sys-plugin defaults are overridden.
                str(core / 'config' / 'mavros_gcs.yaml'),
                {'fcu_url': ParameterValue(LaunchConfiguration('fcu_url'), value_type=str),
                 'gcs_url': '', 'tgt_system': 1, 'tgt_component': 1, 'fcu_protocol': 'v2.0'},
            ],
        ),
    ])
