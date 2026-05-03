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

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _resolve_calib_helper_settings(params_file_path: str, namespace_value: str) -> tuple[str, bool]:
    mode_value = "always"
    no_csv_value = False

    if params_file_path:
        try:
            with open(params_file_path, "r", encoding="utf-8") as file_obj:
                yaml_obj = yaml.safe_load(file_obj) or {}
        except Exception:
            yaml_obj = {}

        if isinstance(yaml_obj, dict):
            root_candidates = [yaml_obj]
            normalized_namespace = namespace_value.strip().lstrip("/")
            if normalized_namespace:
                namespaced_root = yaml_obj.get(normalized_namespace)
                if isinstance(namespaced_root, dict):
                    root_candidates.insert(0, namespaced_root)

            for root in root_candidates:
                switches_obj = root.get("pb_navigation_switches")
                if not isinstance(switches_obj, dict):
                    continue
                ros_params = switches_obj.get("ros__parameters")
                if not isinstance(ros_params, dict):
                    continue

                mode_candidate = ros_params.get("calib_helper_mode")
                if isinstance(mode_candidate, str) and mode_candidate.strip():
                    mode_value = mode_candidate.strip().lower()

                no_csv_candidate = ros_params.get("calib_no_csv")
                if isinstance(no_csv_candidate, bool):
                    no_csv_value = no_csv_candidate
                elif isinstance(no_csv_candidate, str):
                    no_csv_value = no_csv_candidate.strip().lower() in ("1", "true", "yes", "on")

                break

    env_mode = os.environ.get("CALIB_HELPER_MODE", "").strip().lower()
    if env_mode:
        mode_value = env_mode

    env_no_csv = os.environ.get("CALIB_NO_CSV", "").strip().lower()
    if env_no_csv:
        no_csv_value = env_no_csv in ("1", "true", "yes", "on")

    return mode_value, no_csv_value


def _setup_calib_helper(context, *_args, **_kwargs):
    params_file_path = LaunchConfiguration("params_file").perform(context)
    namespace_value = LaunchConfiguration("namespace").perform(context)

    calib_mode, calib_no_csv = _resolve_calib_helper_settings(
        params_file_path=params_file_path,
        namespace_value=namespace_value,
    )
    calib_enabled = calib_mode not in ("0", "false", "no", "off", "disable", "disabled")

    calib_script = os.environ.get("CALIB_HELPER_SCRIPT", "")
    if not calib_script:
        bringup_dir = get_package_share_directory("gxu2026_nav_bringup")
        _ws_candidate = Path(bringup_dir)
        for _ in range(6):
            _ws_candidate = _ws_candidate.parent
            _candidate = _ws_candidate / "scripts" / "calib_point_helper.py"
            if _candidate.exists():
                calib_script = str(_candidate)
                break

    if not (calib_enabled and calib_script and os.path.isfile(calib_script)):
        return []

    calib_args = ["python3", calib_script]
    if calib_no_csv:
        calib_args.append("--no-csv")

    if calib_mode in ("always", "all", "nav"):
        return [
            ExecuteProcess(
                cmd=calib_args,
                output="screen",
            )
        ]

    return [
        ExecuteProcess(
            cmd=calib_args,
            output="screen",
            condition=IfCondition(LaunchConfiguration("slam")),
        )
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("gxu2026_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mid360_driver = LaunchConfiguration("use_mid360_driver")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2026",
        description="Select world map name in map/reality (map file shares the same name as this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "reality", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "reality", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_mid360_driver_cmd = DeclareLaunchArgument(
        "use_mid360_driver",
        default_value="True",
        description="Whether to start mid360 driver. Set False during bag replay to avoid duplicate publisher.",
    )

    # Create our own temporary YAML files that include substitutions

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    start_mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
        condition=IfCondition(use_mid360_driver),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mid360_driver_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_mid360_driver_node)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    ld.add_action(OpaqueFunction(function=_setup_calib_helper))

    return ld