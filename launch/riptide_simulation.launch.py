from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os
import tempfile
import jinja2


def rviz_simulation(context: LaunchContext, riptides):
    # Getting LaunchConfiguration
    start_rviz = LaunchConfiguration("start_rviz")
    description_package = LaunchConfiguration("description_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Check if rviz has to be started
    context.perform_substitution(start_rviz)
    if start_rviz == "false":
        return []
    
    # Getting the path of the file
    path = PathJoinSubstitution([FindPackageShare(description_package), "config"]).perform(context)
    filename = context.perform_substitution(rviz_config_file)
    
    # Checking if the file has to be rendered by jinja2
    if filename.split(".")[-1] == "rviz":
        rendered_rviz_file = os.path.join(path, filename)
    elif filename.split(".")[-1] == "j2":
        # Configuring jinja
        templateLoader = jinja2.FileSystemLoader(searchpath=path)
        templateEnv = jinja2.Environment(loader=templateLoader)
        template = templateEnv.get_template(filename)
        rip = [{"name": r} for r in riptides]
        outputText = template.render(riptides=rip)

        fd, path = tempfile.mkstemp()
        # try:
        with os.fdopen(fd, 'w') as tmp:
            # do stuff with temp file
            tmp.write(outputText)
        # finally:
        #     os.remove(path)
        rendered_rviz_file = path
    else:
        return []

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rendered_rviz_file],
    )

    return [rviz]


def generate_launch_description():
    # LaunchDescription
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="riptide_bringup_sim",
            description='Package with the controller\'s configuration in "config" folder. \
                Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_package",
            default_value="riptide_description_sim",
            description="Description package with robot URDF/xacro files. Usually the argument \
                is not set, it enables use of a custom description.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "teleop",
            default_value='false',
            description="Specifies if the teleop is launched",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="riptide_simulation.j2",
            description="RViz2 configuration file.",
        )
    )
    
    # Initialize Arguments
    launch_teleop = LaunchConfiguration("teleop")

    # Simulator
    ld.add_action(
        Node(
            package="riptide_simulator",
            executable="riptide_simulator",
            output="both",
        )
    )

    # Riptide spawner
    riptides = ["riptide_1"]
    for name in riptides:
        # Spawn riptide simulation
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/riptide_simulation_spawn.launch.py"]),
                launch_arguments={
                    "namespace": name,
                }.items(),
            )
        )

        # Spawn orthogonal controller
        # ld.add_action(
        #     Node(
        #         package="riptide_navigation",
        #         executable="orthogonal_controller",
        #         namespace=name,
        #         output="both"
        #     )
        # )

        # Spawn stable cycles
        # ld.add_action(
        #     Node(
        #         package="riptide_navigation",
        #         executable="stable_cycles",
        #         namespace=name,
        #         output="both"
        #     )
        # )

        # Riptide teleop
        # ld.add_action(
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/riptide_teleop.launch.py"]),
        #         launch_arguments={
        #             "cmd_vel": "/" + name + "/riptide_controller/cmd_vel"
        #         }.items(),
        #         condition=IfCondition(launch_teleop)
        #     )
        # )

    ld.add_action(OpaqueFunction(function=rviz_simulation, args=[riptides]))

    return ld