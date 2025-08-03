from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="my_py_pkg",
        executable="number_publisher",
        namespace="/venky",
        name="my_number_publisher",
        remappings=[("/number", "/my_number")],
        parameters=[
            {"number": 12},
            {"timer_period": 3.5}
        ]
    )

    number_counter = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        namespace="/venky",
        name="my_number_counter",
        remappings=[("/number", "/my_number")]
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)

    return ld