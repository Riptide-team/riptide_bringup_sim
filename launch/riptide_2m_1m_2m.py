from launch import LaunchDescription
from launch_ros.actions.node import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Get URDF via xacro
    prefix = "riptide_1"

    # Controller manager
    navigation_node = Node(
        package="riptide_navigation",
        executable="003-2m_1m_2m.py",
        namespace=prefix,
        output="both",
    )
    ld.add_action(navigation_node)

    # Depth Controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["depth_controller", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    return ld