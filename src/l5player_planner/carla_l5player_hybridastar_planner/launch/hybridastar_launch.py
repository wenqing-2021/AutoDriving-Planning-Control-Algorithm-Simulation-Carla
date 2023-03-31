import launch
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import ament_index_python.packages
import sys

def generate_launch_description():

    # print(sys.argv[0])
    # print(__file__)
    print("****************")

    print(get_package_share_directory('carla_l5player_hybridastar_planner'))
    print(os.getcwd())
    
    hybridastar_parameters_configuration = os.path.join(os.getcwd(), 'src/l5player_planner/carla_l5player_hybridastar_planner/config', 'hybridastar_parameters_configuration.yaml')

    # rviz_config_dir = os.path.join(os.getcwd(), 'src/l5player_planner/carla_l5player_mpc_controller/rviz', 'mpc_vis.rviz')
    print(hybridastar_parameters_configuration)

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='carla_l5player_hybridastar_planner',
            executable='hybridastar_search',
            name='hybridastar_search',
            parameters=[hybridastar_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        # Node(package='rviz2',
        #      executable='rviz2',
        #      output='screen',
        #      arguments=['-d', rviz_config_dir]),
    ])
