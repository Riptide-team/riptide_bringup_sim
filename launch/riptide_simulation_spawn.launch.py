import xacro

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def riptide_spawner(context: LaunchContext):
    # Getting launch arguments
    namespace = LaunchConfiguration("namespace")
    controllers_file = LaunchConfiguration("controllers_file")
    controllers_package = LaunchConfiguration("controllers_package")
    simulation_config_file = LaunchConfiguration("simulation_config_file")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_file = LaunchConfiguration("description_file")
    description_package = LaunchConfiguration("description_package")
    
    xacro_path = PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]).perform(context)

    # getting prefix
    prefix = context.perform_substitution(namespace)

    # Getting robot's urdf from xacro
    xacro_description = xacro.process_file(xacro_path, mappings={"prefix": prefix})
    robot_description = {"robot_description": xacro_description.toxml()}

    # Getting the simulation configuration file path
    simulation_configuration = PathJoinSubstitution([runtime_config_package, "config", simulation_config_file])

    # Riptide spawner
    riptide_spawner = Node(
        package="riptide_simulator",
        executable="riptide_spawner",
        namespace=namespace,
        output="both",
        parameters=[simulation_configuration, robot_description]
    )

    # Control node
    controller_config = PathJoinSubstitution([controllers_package, "config", controllers_file])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, controller_config],
        output="both",
    )

    # Pressure Broadcaster
    pressure_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=prefix,
        arguments=["pressure_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    )

    # Imu Broadcaster
    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=prefix,
        arguments=["imu_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    )

    # Tail Broadcaster
    tail_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=prefix,
        arguments=["tail_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    )

    # Riptide controller
    riptide_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=prefix,
        arguments=["riptide_controller", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    )

    log_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=prefix,
        arguments=["log_controller", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    )

    return [riptide_spawner, control_node, pressure_broadcaster, imu_broadcaster, tail_broadcaster, riptide_controller]


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="riptide_1",
            description="Namespace used for the riptide",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="riptide_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
                Usually the argument is not set, it enables use of a custom setup.'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "simulation_config_file",
            default_value="simulation.yaml",
            description="Configuration YAML file describing riptide's configuration" 
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controller_manager.yaml",
            description="YAML file with the controllers configuration."
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "controllers_package",
            default_value="riptide_bringup_sim",
            description="Riptide controllers YAML file package."
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_file",
            default_value="riptide.urdf.xacro",
            description="URDF riptide description for simulation."
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_package",
            default_value="riptide_description",
            description="Riptide description package."
        )
    )

    # Riptide spawner and controller node
    ld.add_action(OpaqueFunction(function=riptide_spawner)) 
    
    return ld