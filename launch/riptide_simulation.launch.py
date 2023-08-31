import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # LaunchDescription
    ld = LaunchDescription()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        )
    )

    # Riptide SDF file
    riptide_description_sim_package = os.path.join(get_package_share_directory('riptide_description_sim'))
    xacro_file = os.path.join(riptide_description_sim_package, 'urdf', 'riptide.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={"prefix": "riptide_1"})
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    # Robot State Publisher
    ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': doc.toxml(), 'ignore_timestamp': True}]
        )
    )

    # Riptide spawner
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace="riptide_1",
        arguments=['-string', doc.toxml(),
                '-name', 'riptide',
                '-allow_renaming', 'true'],
    )
    ld.add_action(
        spawn_entity
    )

    # World
    world_file = os.path.join(riptide_description_sim_package, "worlds", "infinite_ocean.sdf")

    # Simulation launch
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ' + world_file])]
        )
    )

    # Joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        )
    )
    
    return ld