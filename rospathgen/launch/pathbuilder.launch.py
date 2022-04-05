from ament_index_python import get_package_share_directory
import launch
import launch_ros.actions
import launch.actions
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('rospathgen'), 'cfg', 'paths.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package = "rospathgen",
            executable = "pathservices",
            name = "pathserver",
            output = "screen",
            parameters = [config],
            arguments=['--ros-args','--log-level','DEBUG'],
            respawn = True
        ),
        launch_ros.actions.Node(
            package = "rospathgen",
            executable = "pathbuilder",
            name = "pathbuilder",
            output = "screen",
            parameters = [config]
        )
    ])