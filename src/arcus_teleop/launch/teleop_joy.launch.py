from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    teleop_node = Node(
        package="arcus_teleop",
        executable="teleop_joy",
        name="teleop_joy",
        output="screen"
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
