import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import lifecycle_msgs.msg


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "cmd_vel",
            default_value='/cmd_vel',
            description="Topic's name on which is published the twist control.",
        )
    )

    cmd_vel = LaunchConfiguration("cmd_vel")

    wiimote_node = launch_ros.actions.LifecycleNode(
        package='wiimote',
        executable='wiimote_node',
        namespace='',
        name='wiimote',
        output='screen',
        parameters=[
            FindPackageShare('riptide_bringup').find("riptide_bringup") + '/config/wiimote_params.yaml'
        ]
    )
    configure_wiimote = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/wiimote'),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    ))

    activate_wiimote = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/wiimote'),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
    ))

    teleop_wiimote_node = launch_ros.actions.LifecycleNode(
        package='wiimote',
        executable='teleop_wiimote_node',
        namespace='',
        name='teleop_wiimote',
        output='screen',
        remappings=[
            ('/cmd_vel', cmd_vel)
        ],
        parameters=[
            FindPackageShare('riptide_bringup').find("riptide_bringup") + '/config/teleop_params.yaml'
        ]
    )

    configure_teleop_wiimote = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                '/teleop_wiimote'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        ))

    activate_teleop_wiimote = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                '/teleop_wiimote'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        ))

    on_configure_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=wiimote_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg='wiimote successfully configured. '
                        'Proceeding to configure teleop_wiimote.'),
                configure_teleop_wiimote,
            ]
        ))

    on_configure_teleop_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=teleop_wiimote_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg='teleop_wiimote configured. Proceeding to activate both nodes.'),
                activate_wiimote,
                activate_teleop_wiimote,
            ]
        ))

    on_finalized_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=wiimote_node,
            goal_state='finalized',
            entities=[
                launch.actions.LogInfo(
                    msg='wiimote node shutdown. Shutting down entire launch process'),
                launch.actions.Shutdown(reason='wiimote node was shutdown')
            ]
        )
    )

    return launch.LaunchDescription(
        [launch.actions.DeclareLaunchArgument(name='emulate_tty', default_value='True'),
         wiimote_node,
         teleop_wiimote_node,
         configure_wiimote,
         on_configure_wiimote,
         on_configure_teleop_wiimote,
         on_finalized_wiimote])
