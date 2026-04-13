# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:="robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}"
    ex) robots:="robot3={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707};
                 robot4={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory("gxu2026_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Backward-compatible no-op argument",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description=(
            "Full path to map file to load. Empty means auto-detect exactly one yaml "
            "under map/simulation"
        ),
    )

    def _resolve_single_map_file(context, *, map_arg, map_dir):
        map_override = (map_arg.perform(context) or "").strip()
        if map_override:
            map_path = Path(map_override).expanduser()
            if not map_path.is_file():
                raise RuntimeError(
                    f"Map file does not exist: {map_path}. Navigation launch aborted."
                )
            return [
                SetLaunchConfiguration("map", str(map_path)),
                LogInfo(
                    msg=[
                        "Using map yaml: ",
                        map_path.name,
                        " (",
                        str(map_path),
                        ")",
                    ]
                ),
            ]

        map_candidates = sorted(
            path for path in Path(map_dir).expanduser().glob("*.yaml") if path.is_file()
        )
        if not map_candidates:
            raise RuntimeError(
                f"No map yaml found in {map_dir}. Navigation launch aborted."
            )
        if len(map_candidates) > 1:
            candidates = ", ".join(path.name for path in map_candidates)
            raise RuntimeError(
                f"Expected exactly one map yaml in {map_dir}, found {len(map_candidates)}: {candidates}. Navigation launch aborted."
            )
        selected_map = map_candidates[0]
        return [
            SetLaunchConfiguration("map", str(selected_map)),
            LogInfo(
                msg=[
                    "Using map yaml: ",
                    selected_map.name,
                    " (",
                    str(selected_map),
                    ")",
                ]
            ),
        ]

    resolve_map_cmd = OpaqueFunction(
        function=_resolve_single_map_file,
        kwargs={
            "map_arg": map_yaml_file,
            "map_dir": os.path.join(bringup_dir, "map", "simulation"),
        },
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the stacks",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    robots_list = ParseMultiRobotPose("robots").value()

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                LogInfo(
                    msg=[
                        "Launching namespace=",
                        robot_name,
                        " init_pose=",
                        str(init_pose),
                    ]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "rviz_launch.py")
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot_name),
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            bringup_dir, "launch", "rm_navigation_simulation_launch.py"
                        )
                    ),
                    launch_arguments={
                        "namespace": robot_name,
                        "map": map_yaml_file,
                        "use_sim_time": "True",
                        "params_file": params_file,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "headless": "False",
                        "x_pose": TextSubstitution(text=str(init_pose["x"])),
                        "y_pose": TextSubstitution(text=str(init_pose["y"])),
                        "z_pose": TextSubstitution(text=str(init_pose["z"])),
                        "roll": TextSubstitution(text=str(init_pose["roll"])),
                        "pitch": TextSubstitution(text=str(init_pose["pitch"])),
                        "yaw": TextSubstitution(text=str(init_pose["yaw"])),
                        "robot_name": TextSubstitution(text=robot_name),
                    }.items(),
                ),
            ]
        )

        bringup_cmd_group.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(resolve_map_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(LogInfo(msg=["number_of_robots=", str(len(robots_list))]))

    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=["map yaml: ", map_yaml_file])
    )
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=["params yaml: ", params_file])
    )
    ld.add_action(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=["rviz config file: ", rviz_config_file],
        )
    )
    ld.add_action(
        LogInfo(condition=IfCondition(log_settings), msg=["autostart: ", autostart])
    )

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
