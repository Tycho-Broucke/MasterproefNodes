import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='masterproef_nodes',
            executable='yolo_coordinate_publisher',
            name='yolo_coordinate_publisher_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='masterproef_nodes',
            executable='target_selector_node',
            name='target_selector_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='masterproef_nodes',
            executable='heartbeat_publisher',
            name='heartbeat_publisher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='masterproef_nodes',
            executable='beamer_controller',
            name='beamer_controller',
            output='screen'
        )
    ])
