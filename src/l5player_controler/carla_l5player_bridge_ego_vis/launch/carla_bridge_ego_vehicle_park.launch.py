'''
Author: wenqing-2021 1140349586@qq.com
Date: 2023-03-12 20:13:51
LastEditors: wenqing-2021 1140349586@qq.com
LastEditTime: 2023-03-16 22:18:24
FilePath: /AutoDriving-Planning-Control-Algorithm-Simulation-Carla/src/l5player_controler/carla_l5player_bridge_ego_vis/launch/carla_bridge_ego_vehicle_park.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def launch_carla_spawn_object(context, *args, **kwargs):
    # workaround to use launch argument 'role_name' as a part of the string used for the spawn_point param name
    spawn_point_param_name = 'spawn_point_' + \
        launch.substitutions.LaunchConfiguration('role_name').perform(context)

    carla_spawn_objects_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': get_package_share_directory('carla_spawn_objects') + '/config/objects.json',
            spawn_point_param_name: launch.substitutions.LaunchConfiguration('spawn_point')
        }.items()
    )

    return [carla_spawn_objects_launch]

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='96.6, -207.0, 2.0, 0.0, 0.0, -180'
            # default_value='176.1,-195.1,2,0,0,180'
            # default_value='325,-195.4,2,0,0,180'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='8.33' # in m/s
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sigterm_timeout',
            default_value='15'
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        
        launch.actions.OpaqueFunction(function=launch_carla_spawn_object),
    ])
    
    return ld

if __name__ == '__main__':
    generate_launch_description()