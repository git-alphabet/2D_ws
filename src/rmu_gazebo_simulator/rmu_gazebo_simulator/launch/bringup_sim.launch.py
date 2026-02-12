import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    enable_chassis_odometry_gt = LaunchConfiguration("enable_chassis_odometry_gt")
    use_gui = LaunchConfiguration("use_gui")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
            "use_gui": use_gui,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_path,
            "world": selected_world,
            "enable_chassis_odometry_gt": enable_chassis_odometry_gt,
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "enable_chassis_odometry_gt",
            default_value="true",
            description=(
                "Whether to bridge chassis ground-truth odometry from Gazebo to ROS as '<ns>/chassis_odometry_gt'. "
                "Disable this to test localization/nav behavior without simulator GT odometry."),
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Whether to start Ignition Gazebo with the GUI",
        )
    )

    ld.add_action(gazebo_launch)
    # Delay robot spawning to allow Gazebo rendering engine to fully initialise.
    # Spawning too early triggers a known Ignition Fortress 6 bug where
    # SceneManager::CreateVisual creates duplicate scene nodes and crashes.
    ld.add_action(TimerAction(period=5.0, actions=[spawn_robots_launch]))
    ld.add_action(TimerAction(period=5.0, actions=[referee_system_launch]))

    return ld
