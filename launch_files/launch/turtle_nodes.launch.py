from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="sim"
    )

    turtle_spawner = Node(
        package="turtle_catch_all_py",
        executable="turtle_spawner",
        name="turtle_spawner"
    )

    turtle_controller = Node(
        package="turtle_catch_all_py",
        executable="turtle_controller",
        name="turtle_controller"
    )

    ld.add_action(turtlesim)
    ld.add_action(turtle_spawner)
    ld.add_action(turtle_controller)

    return ld