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

import yaml  # type: ignore

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("gxu2026_nav_bringup")

    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    odometry_source = LaunchConfiguration("odometry_source")

    # Variables
    lifecycle_nodes = ["map_saver"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_odometry_source_cmd = DeclareLaunchArgument(
        "odometry_source",
        default_value="",
        description="Override odometry source (point_lio or small_point_lio). Empty means read from pb_navigation_switches.odometry_source.",
    )

    def _set_slam_switches(
        context, *, params_file, namespace, odometry_source
    ):
        override = odometry_source.perform(context).strip()
        if override and override.lower() not in {"auto", "default"}:
            return [SetLaunchConfiguration("odometry_source", override)]

        params_path = Path(params_file.perform(context)).expanduser()
        ns_value = namespace.perform(context)
        odometry_selection = "point_lio"
        if params_path.is_file():
            try:
                raw_data = yaml.safe_load(params_path.read_text()) or {}
                if ns_value and ns_value in raw_data:
                    raw_data = raw_data.get(ns_value, {}) or {}
                switches = (raw_data.get("pb_navigation_switches", {}) or {}).get(
                    "ros__parameters", {}
                )
                odom_source_raw = switches.get("odometry_source")
                if isinstance(odom_source_raw, str) and odom_source_raw.strip():
                    odometry_selection = odom_source_raw.strip()
                else:
                    legacy_enable_small = switches.get("enable_small_point_lio")
                    if isinstance(legacy_enable_small, bool) and legacy_enable_small:
                        odometry_selection = "small_point_lio"
            except Exception:
                odometry_selection = "point_lio"

        return [SetLaunchConfiguration("odometry_source", odometry_selection)]

    set_switches_cmd = OpaqueFunction(
        function=_set_slam_switches,
        kwargs={
            "params_file": params_file,
            "namespace": namespace,
            "odometry_source": odometry_source,
        },
    )

    start_map_saver_server_cmd = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_sync_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(
            PythonExpression(["'", odometry_source, "' == 'point_lio'"])
        ),
    )

    start_small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(
            PythonExpression(["'", odometry_source, "' == 'small_point_lio'"])
        ),
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_odometry_source_cmd)

    # Set switches before starting nodes
    ld.add_action(set_switches_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_small_point_lio_node)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)

    return ld
