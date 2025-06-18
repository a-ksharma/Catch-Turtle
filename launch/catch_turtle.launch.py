from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    spawner= Node(
        package = "catch_turtle",
        executable = "turtle_spawner"
    )
    controller = Node(
        package = "catch_turtle",
        executable = "turtle_controller"
    )

    ld.add_action(spawner)
    ld.add_action(controller)
    return ld