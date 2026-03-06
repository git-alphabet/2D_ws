from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mid360_driver"),
                    "config",
                    "param.yaml",
                ]
            )
        ],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("mid360_driver"),
            "config",
            "display_point_cloud.rviz",
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["--display-config", rviz_config],
    )

    return LaunchDescription([mid360_driver_node, rviz2])
