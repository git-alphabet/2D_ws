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


import copy
import math
import os
import tempfile
from pathlib import Path

import yaml  # type: ignore

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("gxu2026_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    processed_params_file = LaunchConfiguration("processed_params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    terrain_registered_scan_topic = LaunchConfiguration("terrain_registered_scan_topic")
    terrain_lidar_odometry_topic = LaunchConfiguration("terrain_lidar_odometry_topic")
    sensor_scan_registered_scan_topic = LaunchConfiguration("sensor_scan_registered_scan_topic")
    sensor_scan_lidar_odometry_topic = LaunchConfiguration("sensor_scan_lidar_odometry_topic")
    point_lio_config_file = LaunchConfiguration("point_lio_config_file")


    enable_gimbal_yaw_bridge = LaunchConfiguration("enable_gimbal_yaw_bridge")
    enable_rm_behavior_tree = LaunchConfiguration("enable_rm_behavior_tree")
    rm_behavior_tree_style_path = LaunchConfiguration("rm_behavior_tree_style_path")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

    # Normalize namespace for YAML root key (strip leading '/'; treat '/' as empty).
    normalized_root_key = PythonExpression(["'", namespace, "'.lstrip('/')"])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=processed_params_file,
            root_key=normalized_root_key,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether SLAM mode is enabled. Used to avoid duplicate obstacle_scan publishers.",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    nonlinear_spin_publisher_node = Node(
        package="fake_vel_transform",
        executable="nonlinear_spin_publisher",
        name="nonlinear_spin_publisher",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_terrain_registered_scan_topic_cmd = DeclareLaunchArgument(
        "terrain_registered_scan_topic",
        default_value="",
        description=(
            "Override terrain input point cloud topic. Empty means auto from params/switches"
        ),
    )

    declare_terrain_lidar_odometry_topic_cmd = DeclareLaunchArgument(
        "terrain_lidar_odometry_topic",
        default_value="",
        description=(
            "Override terrain input odometry topic. Empty means auto from params/switches"
        ),
    )

    declare_sensor_scan_registered_scan_topic_cmd = DeclareLaunchArgument(
        "sensor_scan_registered_scan_topic",
        default_value="",
        description=(
            "Override sensor_scan_generation point cloud input topic. Empty means auto from params/switches"
        ),
    )

    declare_sensor_scan_lidar_odometry_topic_cmd = DeclareLaunchArgument(
        "sensor_scan_lidar_odometry_topic",
        default_value="",
        description=(
            "Override sensor_scan_generation odometry input topic. Empty means auto from params/switches"
        ),
    )

    declare_point_lio_config_file_cmd = DeclareLaunchArgument(
        "point_lio_config_file",
        default_value=os.path.join(
            get_package_share_directory("point_lio"), "config", "mid360.yaml"
        ),
        description="Full path to point_lio config file for mid360 obstacle supplement chain",
    )

    enable_obstacle_scan = LaunchConfiguration("enable_obstacle_scan")
    enable_scan_additive = LaunchConfiguration("enable_scan_additive")
    obstacle_scan_output_topic = LaunchConfiguration("obstacle_scan_output_topic")

    start_pointcloud_to_laserscan_cmd = Node(
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
            ("scan", obstacle_scan_output_topic),
        ],
        condition=IfCondition(enable_obstacle_scan),
    )

    load_pointcloud_to_laserscan_composable_cmd = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_composition,
                    "' == 'True') and ('",
                    enable_obstacle_scan,
                    "' == 'true')",
                ]
            )
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan",
                parameters=[configured_params],
                remappings=[
                    ("cloud_in", "terrain_map_ext"),
                    ("scan", obstacle_scan_output_topic),
                ],
            )
        ],
    )

    start_point_lio_cmd = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio_mid360",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[point_lio_config_file],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("livox/lidar", "/livox/lidar"),
            ("livox/imu", "/livox/imu"),
        ],
        condition=IfCondition(enable_scan_additive),
    )

    start_terrain_analysis_ext_mid360_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext_mid360",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ("registered_scan", "registered_scan"),
            ("lidar_odometry", "lidar_odometry"),
            ("terrain_map_ext", "terrain_map_ext_mid360"),
        ],
        condition=IfCondition(enable_scan_additive),
    )

    start_pointcloud_to_laserscan_mid360_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_mid360",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext_mid360"),
            ("scan", "scan_mid360"),
        ],
        condition=IfCondition(enable_scan_additive),
    )

    start_scan_additive_adapter_cmd = Node(
        package="scan_additive_adapter",
        executable="scan_additive_adapter_node",
        name="scan_additive_adapter",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {
                "primary_scan_topic": "scan_odin1",
                "secondary_scan_topic": "scan_mid360",
                "output_scan_topic": "obstacle_scan",
                "secondary_timeout_sec": 0.3,
                "publish_secondary_when_primary_missing": True,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(enable_scan_additive),
    )

    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ("registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
        ],
    )

    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ("registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
        ],
    )

    start_rm_behavior_tree_cmd = Node(
        package="rm_behavior_tree",
        executable="rm_behavior_tree",
        name="rm_behavior_tree",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {"style": rm_behavior_tree_style_path}],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(enable_rm_behavior_tree),
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("registered_scan", sensor_scan_registered_scan_topic),
                    ("lidar_odometry", sensor_scan_lidar_odometry_topic),
                ],
            ),
            start_pointcloud_to_laserscan_cmd,
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                name="fake_vel_transform",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=[configured_params],
                remappings=[
                    ("registered_scan", sensor_scan_registered_scan_topic),
                    ("lidar_odometry", sensor_scan_lidar_odometry_topic),
                ],
            ),
            ComposableNode(
                package="fake_vel_transform",
                plugin="fake_vel_transform::FakeVelTransform",
                name="fake_vel_transform",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    def _set_navigation_switches(
        context,
        *,
        params_file,
        namespace,
        slam,
        use_sim_time,
        terrain_registered_scan_topic,
        terrain_lidar_odometry_topic,
        sensor_scan_registered_scan_topic,
        sensor_scan_lidar_odometry_topic,
    ):
        params_path = Path(params_file.perform(context)).expanduser()
        ns_value = namespace.perform(context)
        default_style_file = "rmuc_01.xml"
        enable_rm_bt = False
        style_file = default_style_file
        processed_file = str(params_path)
        controller_plugin_name = None
        neupan_frame_name = None
        enable_obstacle_scan_value = "false"
        enable_scan_additive_value = "false"
        obstacle_scan_output_topic_value = "obstacle_scan"
        enable_gimbal_yaw_bridge_value = False
        terrain_registered_scan_topic_value = "registered_scan"
        terrain_lidar_odometry_topic_value = "lidar_odometry"
        sensor_scan_registered_scan_topic_value = "registered_scan"
        sensor_scan_lidar_odometry_topic_value = "lidar_odometry"

        slam_raw = slam.perform(context)
        slam_enabled = str(slam_raw).strip().lower() in {"true", "1", "yes", "on"}
        sim_raw = use_sim_time.perform(context)
        sim_enabled = str(sim_raw).strip().lower() in {"true", "1", "yes", "on"}
        terrain_registered_scan_topic_override = (
            terrain_registered_scan_topic.perform(context) or ""
        ).strip()
        terrain_lidar_odometry_topic_override = (
            terrain_lidar_odometry_topic.perform(context) or ""
        ).strip()
        sensor_scan_registered_scan_topic_override = (
            sensor_scan_registered_scan_topic.perform(context) or ""
        ).strip()
        sensor_scan_lidar_odometry_topic_override = (
            sensor_scan_lidar_odometry_topic.perform(context) or ""
        ).strip()

        def _resolve_bt_style_path(style_value):
            candidate = style_value.strip() if isinstance(style_value, str) else ""
            if not candidate:
                candidate = default_style_file

            if candidate.startswith("$("):
                return candidate

            expanded_candidate = os.path.expanduser(candidate)
            if os.path.isabs(expanded_candidate):
                return expanded_candidate

            package_name = None
            relative_path = expanded_candidate
            if ":" in expanded_candidate:
                pkg_part, rel_part = expanded_candidate.split(":", 1)
                pkg_part = pkg_part.strip()
                if pkg_part:
                    package_name = pkg_part
                    relative_path = rel_part.lstrip("/") or default_style_file

            try:
                share_dir = get_package_share_directory(package_name or "rm_behavior_tree")
            except PackageNotFoundError:
                share_dir = get_package_share_directory("rm_behavior_tree")

            if package_name:
                return os.path.join(share_dir, relative_path)

            if relative_path.startswith("./") or relative_path.startswith("../"):
                return os.path.normpath(
                    os.path.join(str(params_path.parent), relative_path)
                )

            if os.path.sep in relative_path:
                return os.path.join(share_dir, relative_path)

            return os.path.join(share_dir, "config", relative_path)

        if params_path.is_file():
            try:
                raw_yaml = yaml.safe_load(params_path.read_text()) or {}
            except Exception:
                raw_yaml = {}

            target_data = raw_yaml
            if ns_value:
                maybe_namespaced = raw_yaml.get(ns_value)
                if isinstance(maybe_namespaced, dict):
                    target_data = maybe_namespaced

            def _get_ros_params(container, key):
                entry = container.get(key) if isinstance(container, dict) else None
                if isinstance(entry, dict):
                    params = entry.get("ros__parameters")
                    if isinstance(params, dict):
                        return params
                return {}

            def _set_nested_value(container, key_path, value):
                current = container
                for key in key_path[:-1]:
                    if not isinstance(current, dict):
                        return
                    current = current.get(key)
                    if current is None:
                        return
                if isinstance(current, dict):
                    current[key_path[-1]] = value

            def _normalize_costmap_size_types(container):
                """兼容配置文件里 width/height 写成 20 或 20.0 的情况。"""
                changed = False
                param_paths = [
                    ["local_costmap", "local_costmap", "ros__parameters"],
                    ["global_costmap", "global_costmap", "ros__parameters"],
                ]

                for path in param_paths:
                    current = container
                    for key in path:
                        if not isinstance(current, dict):
                            current = None
                            break
                        current = current.get(key)
                    if not isinstance(current, dict):
                        continue

                    for field in ("width", "height"):
                        raw_value = current.get(field)
                        if isinstance(raw_value, float):
                            if math.isfinite(raw_value) and raw_value.is_integer():
                                current[field] = int(raw_value)
                                changed = True
                            continue

                        if isinstance(raw_value, str):
                            stripped = raw_value.strip()
                            if not stripped:
                                continue
                            try:
                                parsed = float(stripped)
                            except ValueError:
                                continue
                            if math.isfinite(parsed) and parsed.is_integer():
                                current[field] = int(parsed)
                                changed = True

                return changed

            params_normalized = _normalize_costmap_size_types(target_data)

            switches = _get_ros_params(target_data, "pb_navigation_switches")
            if not switches and target_data is not raw_yaml:
                switches = _get_ros_params(raw_yaml, "pb_navigation_switches")
            enable_rm_bt = bool(switches.get("enable_rm_behavior_tree", enable_rm_bt))

            switches_terrain_registered_scan_topic = switches.get(
                "terrain_registered_scan_topic"
            )
            if isinstance(switches_terrain_registered_scan_topic, str):
                switches_terrain_registered_scan_topic = (
                    switches_terrain_registered_scan_topic.strip()
                )
            else:
                switches_terrain_registered_scan_topic = ""

            switches_terrain_lidar_odometry_topic = switches.get(
                "terrain_lidar_odometry_topic"
            )
            if isinstance(switches_terrain_lidar_odometry_topic, str):
                switches_terrain_lidar_odometry_topic = (
                    switches_terrain_lidar_odometry_topic.strip()
                )
            else:
                switches_terrain_lidar_odometry_topic = ""

            switches_sensor_scan_registered_scan_topic = switches.get(
                "sensor_scan_registered_scan_topic"
            )
            if isinstance(switches_sensor_scan_registered_scan_topic, str):
                switches_sensor_scan_registered_scan_topic = (
                    switches_sensor_scan_registered_scan_topic.strip()
                )
            else:
                switches_sensor_scan_registered_scan_topic = ""

            switches_sensor_scan_lidar_odometry_topic = switches.get(
                "sensor_scan_lidar_odometry_topic"
            )
            if isinstance(switches_sensor_scan_lidar_odometry_topic, str):
                switches_sensor_scan_lidar_odometry_topic = (
                    switches_sensor_scan_lidar_odometry_topic.strip()
                )
            else:
                switches_sensor_scan_lidar_odometry_topic = ""

            odometry_source = switches.get("odometry_source")
            if isinstance(odometry_source, str):
                odometry_source = odometry_source.strip().lower()
            else:
                odometry_source = ""

            if switches_terrain_registered_scan_topic:
                terrain_registered_scan_topic_value = switches_terrain_registered_scan_topic
            elif odometry_source == "odin1" and not sim_enabled:
                terrain_registered_scan_topic_value = "odin1/cloud_slam"

            if switches_terrain_lidar_odometry_topic:
                terrain_lidar_odometry_topic_value = switches_terrain_lidar_odometry_topic
            elif odometry_source == "odin1" and not sim_enabled:
                terrain_lidar_odometry_topic_value = "odin1/odometry_highfreq"

            if switches_sensor_scan_registered_scan_topic:
                sensor_scan_registered_scan_topic_value = (
                    switches_sensor_scan_registered_scan_topic
                )
            elif odometry_source == "odin1" and not sim_enabled:
                sensor_scan_registered_scan_topic_value = "odin1/cloud_slam"

            if switches_sensor_scan_lidar_odometry_topic:
                sensor_scan_lidar_odometry_topic_value = (
                    switches_sensor_scan_lidar_odometry_topic
                )
            elif odometry_source == "odin1" and not sim_enabled:
                sensor_scan_lidar_odometry_topic_value = "odin1/odometry_highfreq"

            # 收敛接口：只暴露一个开关 enable_gimbal_yaw_bridge。
            # 兼容旧配置：enable_auto_aim_yaw_bridge / enable_auto_aim_yaw_sim_pub。
            if "enable_gimbal_yaw_bridge" in switches:
                enable_gimbal_yaw_bridge_value = bool(switches.get("enable_gimbal_yaw_bridge"))
            else:
                legacy_bridge = bool(switches.get("enable_auto_aim_yaw_bridge", False))
                legacy_sim_pub = bool(switches.get("enable_auto_aim_yaw_sim_pub", False))
                enable_gimbal_yaw_bridge_value = legacy_bridge or legacy_sim_pub

            raw_frame_name = switches.get("neupan_fake_frame")
            if isinstance(raw_frame_name, str):
                stripped_name = raw_frame_name.strip()
                if stripped_name:
                    neupan_frame_name = stripped_name

            plugin_from_params = switches.get("controller_plugin")
            if isinstance(plugin_from_params, str):
                plugin_candidate = plugin_from_params.strip()
                if plugin_candidate:
                    controller_plugin_name = plugin_candidate

            behavior_tree_selector = switches.get("behavior_tree")
            if isinstance(behavior_tree_selector, str):
                behavior_tree_selector = behavior_tree_selector.strip()
            else:
                behavior_tree_selector = None

            rm_bt_params = _get_ros_params(target_data, "rm_behavior_tree")
            if not rm_bt_params and target_data is not raw_yaml:
                rm_bt_params = _get_ros_params(raw_yaml, "rm_behavior_tree")
            style_file = rm_bt_params.get("style", style_file)
            if behavior_tree_selector:
                selector_lower = behavior_tree_selector.lower()
                if selector_lower in {"disabled", "none", "nav2", "default"}:
                    enable_rm_bt = False
                else:
                    enable_rm_bt = True
                    style_file = behavior_tree_selector
            else:
                enable_rm_bt = enable_rm_bt and bool(rm_bt_params)

            controller_server = target_data.setdefault("controller_server", {}).setdefault(
                "ros__parameters", {}
            )
            controller_plugins = controller_server.get("controller_plugins")
            if not controller_plugins:
                controller_plugins = ["FollowPath"]
                controller_server["controller_plugins"] = controller_plugins
            active_plugin_slot = controller_plugins[0]

            available_profiles = {}
            default_profile = controller_server.get(active_plugin_slot)
            default_plugin_key = None

            def _plugin_key_from_profile(profile_dict):
                plugin_field = profile_dict.get("plugin") if isinstance(profile_dict, dict) else None
                if isinstance(plugin_field, str) and plugin_field:
                    return plugin_field.split("::", 1)[0]
                return None

            if isinstance(default_profile, dict):
                default_plugin_key = _plugin_key_from_profile(default_profile)
                if default_plugin_key:
                    available_profiles[default_plugin_key] = copy.deepcopy(
                        default_profile
                    )

            additional_profiles = _get_ros_params(target_data, "pb_controller_profiles")
            if not additional_profiles and target_data is not raw_yaml:
                additional_profiles = _get_ros_params(raw_yaml, "pb_controller_profiles")
            if isinstance(additional_profiles, dict):
                for name, profile in additional_profiles.items():
                    if isinstance(profile, dict):
                        profile_copy = copy.deepcopy(profile)
                        available_profiles[name] = profile_copy
                        plugin_key = _plugin_key_from_profile(profile_copy)
                        if plugin_key:
                            available_profiles.setdefault(plugin_key, profile_copy)

            selected_plugin_key = controller_plugin_name or default_plugin_key
            if not selected_plugin_key:
                if available_profiles:
                    selected_plugin_key = next(iter(available_profiles))

            # 若选中 neupan_nav2_controller，无条件从 neupan_nav2_controller 包内
            # 按 sim/reality 加载 neupan.yaml，作为 FollowPath 的配置来源。
            # bringup nav2_params.yaml 中不应再有 FollowPath NeuPAN 块；即使残留也会被覆盖。
            if selected_plugin_key and selected_plugin_key.startswith("neupan_nav2_controller"):
                try:
                    neupan_pkg_dir = get_package_share_directory("neupan_nav2_controller")
                    is_sim = str(use_sim_time.perform(context)).strip().lower() in {"true", "1"}
                    sub_dir = "simulation" if is_sim else "reality"
                    neupan_yaml_path = os.path.join(neupan_pkg_dir, "config", sub_dir, "neupan.yaml")
                    with open(neupan_yaml_path, "r") as _f:
                        _neupan_data = yaml.safe_load(_f) or {}
                    _cs_params = _neupan_data.get("controller_server", {}).get("ros__parameters", {})
                    _neupan_profile = _cs_params.get(active_plugin_slot)
                    if isinstance(_neupan_profile, dict):
                        available_profiles[selected_plugin_key] = copy.deepcopy(_neupan_profile)
                        import sys as _sys
                        print(
                            f"[navigation_launch] Loaded NeuPAN profile from {neupan_yaml_path}",
                            file=_sys.stderr,
                        )
                except Exception as _e:
                    import sys as _sys
                    print(
                        f"[navigation_launch] Warning: could not load neupan.yaml: {_e}",
                        file=_sys.stderr,
                    )

            

            frame_override_paths = [
                ["bt_navigator", "ros__parameters", "robot_base_frame"],
                ["local_costmap", "local_costmap", "ros__parameters", "robot_base_frame"],
                ["global_costmap", "global_costmap", "ros__parameters", "robot_base_frame"],
                ["behavior_server", "ros__parameters", "robot_base_frame"],
            ]

            override_required = params_normalized
            if selected_plugin_key and selected_plugin_key in available_profiles:
                target_profile = available_profiles[selected_plugin_key]
                plugin_field = (
                    target_profile.get("plugin") if isinstance(target_profile, dict) else ""
                )
                current_profile = controller_server.get(active_plugin_slot)
                if current_profile != target_profile:
                    controller_server[active_plugin_slot] = copy.deepcopy(target_profile)
                    override_required = True

                neupan_plugin_selected = selected_plugin_key == "neupan_nav2_controller" or (
                    isinstance(plugin_field, str)
                    and plugin_field.startswith("neupan_nav2_controller")
                )
                if neupan_plugin_selected and neupan_frame_name:
                    for path in frame_override_paths:
                        _set_nested_value(target_data, path, neupan_frame_name)
                    override_required = True

                if neupan_plugin_selected and not slam_enabled:
                    enable_obstacle_scan_value = "true"
                    enable_scan_additive_value = "true"
                    obstacle_scan_output_topic_value = "scan_odin1"

            if override_required:
                with tempfile.NamedTemporaryFile(
                    mode="w", delete=False, suffix=".yaml"
                ) as tmp_file:
                    yaml.safe_dump(raw_yaml, tmp_file, default_flow_style=False)
                    processed_file = tmp_file.name

        if terrain_registered_scan_topic_override:
            terrain_registered_scan_topic_value = terrain_registered_scan_topic_override

        if terrain_lidar_odometry_topic_override:
            terrain_lidar_odometry_topic_value = terrain_lidar_odometry_topic_override

        if sensor_scan_registered_scan_topic_override:
            sensor_scan_registered_scan_topic_value = (
                sensor_scan_registered_scan_topic_override
            )

        if sensor_scan_lidar_odometry_topic_override:
            sensor_scan_lidar_odometry_topic_value = (
                sensor_scan_lidar_odometry_topic_override
            )

        style_path = _resolve_bt_style_path(style_file) if enable_rm_bt else style_file
        return [
            SetLaunchConfiguration(
                "enable_rm_behavior_tree", "true" if enable_rm_bt else "false"
            ),
            SetLaunchConfiguration("rm_behavior_tree_style_path", style_path),
            SetLaunchConfiguration("processed_params_file", processed_file),
            SetLaunchConfiguration("enable_obstacle_scan", enable_obstacle_scan_value),
            SetLaunchConfiguration("enable_scan_additive", enable_scan_additive_value),
            SetLaunchConfiguration(
                "obstacle_scan_output_topic", obstacle_scan_output_topic_value
            ),
            SetLaunchConfiguration(
                "enable_gimbal_yaw_bridge",
                "true" if enable_gimbal_yaw_bridge_value else "false",
            ),
            SetLaunchConfiguration(
                "terrain_registered_scan_topic", terrain_registered_scan_topic_value
            ),
            SetLaunchConfiguration(
                "terrain_lidar_odometry_topic", terrain_lidar_odometry_topic_value
            ),
            SetLaunchConfiguration(
                "sensor_scan_registered_scan_topic",
                sensor_scan_registered_scan_topic_value,
            ),
            SetLaunchConfiguration(
                "sensor_scan_lidar_odometry_topic",
                sensor_scan_lidar_odometry_topic_value,
            ),
            # Backward-compatible launch configurations (not used in this file anymore).
            SetLaunchConfiguration(
                "enable_auto_aim_yaw_bridge",
                "true" if enable_gimbal_yaw_bridge_value else "false",
            ),
            SetLaunchConfiguration(
                "enable_auto_aim_yaw_sim_pub",
                "true" if enable_gimbal_yaw_bridge_value else "false",
            ),
        ]

    set_switches_cmd = OpaqueFunction(
        function=_set_navigation_switches,
        kwargs={
            "params_file": params_file,
            "namespace": namespace,
            "slam": slam,
            "use_sim_time": use_sim_time,
            "terrain_registered_scan_topic": terrain_registered_scan_topic,
            "terrain_lidar_odometry_topic": terrain_lidar_odometry_topic,
            "sensor_scan_registered_scan_topic": sensor_scan_registered_scan_topic,
            "sensor_scan_lidar_odometry_topic": sensor_scan_lidar_odometry_topic,
        },
    )

    start_auto_aim_yaw_joint_state_bridge_cmd = Node(
        condition=IfCondition(enable_gimbal_yaw_bridge),
        package="gimbal_yaw_bridge",
        executable="auto_aim_yaw_joint_state_bridge",
        name="auto_aim_yaw_joint_state_bridge",
        output="screen",
        parameters=[configured_params],
    )

    start_auto_aim_yaw_sim_pub_cmd = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    enable_gimbal_yaw_bridge,
                    "' == 'true') and ('",
                    use_sim_time,
                    "' == 'true')",
                ]
            )
        ),
        package="gimbal_yaw_bridge",
        executable="gimbal_state_to_auto_aim_yaw",
        name="gimbal_state_to_auto_aim_yaw",
        output="screen",
        parameters=[configured_params],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_terrain_registered_scan_topic_cmd)
    ld.add_action(declare_terrain_lidar_odometry_topic_cmd)
    ld.add_action(declare_sensor_scan_registered_scan_topic_cmd)
    ld.add_action(declare_sensor_scan_lidar_odometry_topic_cmd)
    ld.add_action(declare_point_lio_config_file_cmd)
    # processed params defaults to original params file
    ld.add_action(SetLaunchConfiguration("processed_params_file", params_file))
    ld.add_action(SetLaunchConfiguration("enable_obstacle_scan", "false"))
    ld.add_action(SetLaunchConfiguration("enable_scan_additive", "false"))
    ld.add_action(SetLaunchConfiguration("obstacle_scan_output_topic", "obstacle_scan"))
    ld.add_action(SetLaunchConfiguration("terrain_registered_scan_topic", "registered_scan"))
    ld.add_action(SetLaunchConfiguration("terrain_lidar_odometry_topic", "lidar_odometry"))
    ld.add_action(SetLaunchConfiguration("sensor_scan_registered_scan_topic", "registered_scan"))
    ld.add_action(SetLaunchConfiguration("sensor_scan_lidar_odometry_topic", "lidar_odometry"))
    # Set switches before starting nodes
    ld.add_action(set_switches_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(nonlinear_spin_publisher_node)
    ld.add_action(start_auto_aim_yaw_sim_pub_cmd)
    ld.add_action(start_auto_aim_yaw_joint_state_bridge_cmd)
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    ld.add_action(start_point_lio_cmd)
    ld.add_action(start_terrain_analysis_ext_mid360_cmd)
    ld.add_action(start_pointcloud_to_laserscan_mid360_cmd)
    ld.add_action(start_scan_additive_adapter_cmd)
    ld.add_action(start_rm_behavior_tree_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_pointcloud_to_laserscan_composable_cmd)

    return ld
