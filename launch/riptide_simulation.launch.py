import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # LaunchDescription
    ld = LaunchDescription()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        )
    )


    # Riptide SDF file
    riptide_description_sim_package = os.path.join(get_package_share_directory('riptide_description_sim'))
    model_file = os.path.join(riptide_description_sim_package, 'model.sdf')

    # Riptide spawner
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-file', model_file,
                    '-name', 'riptide',
                    '-allow_renaming', 'true'],
        )
    )

    # World
    world_file = os.path.join(riptide_description_sim_package, "worlds", "infinite_ocean.sdf")

    # Simulation launch
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ' + world_file])]
        )
    )
    
    return ld