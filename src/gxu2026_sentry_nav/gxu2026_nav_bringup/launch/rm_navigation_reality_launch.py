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
import re
from pathlib import Path

import yaml  # type: ignore

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("gxu2026_nav_bringup")
    odin_driver_dir = get_package_share_directory("odin_ros_driver")
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
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    odin_config_file = LaunchConfiguration("odin_config_file")
    use_odin_driver = LaunchConfiguration("use_odin_driver")
    terrain_registered_scan_topic = LaunchConfiguration("terrain_registered_scan_topic")
    terrain_lidar_odometry_topic = LaunchConfiguration("terrain_lidar_odometry_topic")
    sensor_scan_registered_scan_topic = LaunchConfiguration("sensor_scan_registered_scan_topic")
    sensor_scan_lidar_odometry_topic = LaunchConfiguration("sensor_scan_lidar_odometry_topic")
    robot_name = LaunchConfiguration("robot_name")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will enable mapping mode and publish static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description=(
            "Full path to map file to load. Empty means auto-detect exactly one yaml "
            "under map/reality"
        ),
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

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description=(
            "Robot xmacro basename override. Empty means reading "
            "robot_description_runtime.robot_description_file (fallback: "
            "pb_navigation_switches.robot_description_file) from params_file."
        ),
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use (odin_reality.rviz for debug view)",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_foxglove_cmd = DeclareLaunchArgument(
        "use_foxglove",
        default_value="False",
        description="Whether to start foxglove_bridge for remote visualization",
    )

    declare_foxglove_topics_file_cmd = DeclareLaunchArgument(
        "foxglove_topics_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "foxglove_topics.txt"
        ),
        description="Plain text file with one topic per line (# = comment). Auto-wrapped as regex.",
    )

    declare_odin_config_file_cmd = DeclareLaunchArgument(
        "odin_config_file",
        default_value=os.path.join(odin_driver_dir, "config", "control_command.yaml"),
        description="Full path to odin_ros_driver control config file",
    )

    declare_use_odin_driver_cmd = DeclareLaunchArgument(
        "use_odin_driver",
        default_value="True",
        description="Whether to start odin_ros_driver. Set False during bag replay to avoid duplicate publisher.",
    )

    declare_terrain_registered_scan_topic_cmd = DeclareLaunchArgument(
        "terrain_registered_scan_topic",
        default_value="odin1/cloud_slam",
        description="Optional override for terrain point cloud input topic",
    )

    declare_terrain_lidar_odometry_topic_cmd = DeclareLaunchArgument(
        "terrain_lidar_odometry_topic",
        default_value="odin1/odometry",
        description="Optional override for terrain odometry input topic",
    )

    declare_sensor_scan_registered_scan_topic_cmd = DeclareLaunchArgument(
        "sensor_scan_registered_scan_topic",
        default_value="odin1/cloud_slam",
        description="Optional override for sensor_scan_generation point cloud input topic",
    )

    declare_sensor_scan_lidar_odometry_topic_cmd = DeclareLaunchArgument(
        "sensor_scan_lidar_odometry_topic",
        default_value="odin1/odometry",
        description="Optional override for sensor_scan_generation odometry input topic",
    )

    declare_odin_map_mode_cmd = DeclareLaunchArgument(
        "odin_map_mode",
        default_value="0",
        description=(
            "Odin1 custom_map_mode: "
            "0=Odometry(no map TF, need static TF), "
            "1=SLAM(odin publishes map->odom TF), "
            "2=Relocalization(odin publishes map->odom TF after match). "
            "Must match control_command.yaml custom_map_mode!"
        ),
    )

    # Mode0: 需要我们发 static identity map->odom TF。
    # slam=True 时在 mode1 下仍保留静态兜底，避免设备初始化阶段 map TF 迟到。
    # mode2(重定位)由 odin 成功匹配后发布 map->odom，禁止静态 map->odom 避免重复发布冲突。
    from launch.substitutions import PythonExpression
    publish_static_map_tf = PythonExpression(
        [
            "'True' if ('",
            LaunchConfiguration("odin_map_mode"),
            "' == '0') or (('",
            LaunchConfiguration("slam"),
            "'.lower() in ['true', '1', 'yes', 'on']) and ('",
            LaunchConfiguration("odin_map_mode"),
            "' != '2')) else 'False'",
        ]
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

    def _resolve_single_map_file(context, *, map_arg, slam_arg, odin_mode_arg, map_dir):
        slam_value = (slam_arg.perform(context) or "").strip().lower()
        if slam_value in {"true", "1", "yes", "on"}:
            return []

        odin_mode_value = (odin_mode_arg.perform(context) or "").strip()

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
            if odin_mode_value == "2":
                return [
                    SetLaunchConfiguration("slam", "True"),
                    LogInfo(
                        msg=(
                            "No map yaml found in bringup map dir, but odin_map_mode=2 "
                            "(relocalization). Fallback to slam:=True for startup compatibility."
                        )
                    ),
                ]
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
            "slam_arg": slam,
            "odin_mode_arg": LaunchConfiguration("odin_map_mode"),
            "map_dir": os.path.join(bringup_dir, "map", "reality"),
        },
    )

    def _optional_bool(raw_value):
        if isinstance(raw_value, bool):
            return raw_value
        if isinstance(raw_value, (int, float)):
            return bool(raw_value)
        if isinstance(raw_value, str):
            normalized = raw_value.strip().lower()
            if normalized in {"true", "1", "yes", "on"}:
                return True
            if normalized in {"false", "0", "no", "off"}:
                return False
        return None

    def _set_robot_name_from_params(context, *, params_file, namespace, robot_name):
        default_robot_name = "gxu2026_sentry_robot"
        selected_robot_name = (robot_name.perform(context) or "").strip()

        if not selected_robot_name:
            params_path = Path(params_file.perform(context)).expanduser()
            namespace_value = (namespace.perform(context) or "").strip().lstrip("/")

            if params_path.is_file():
                try:
                    raw_yaml = yaml.safe_load(params_path.read_text()) or {}
                except Exception:
                    raw_yaml = {}

                target_data = raw_yaml
                if namespace_value:
                    namespaced_data = raw_yaml.get(namespace_value)
                    if isinstance(namespaced_data, dict):
                        target_data = namespaced_data

                def _get_ros_params(container, key):
                    entry = container.get(key) if isinstance(container, dict) else None
                    if isinstance(entry, dict):
                        params = entry.get("ros__parameters")
                        if isinstance(params, dict):
                            return params
                    return {}

                def _get_ros_params_with_fallback(key):
                    params = _get_ros_params(target_data, key)
                    if not params and target_data is not raw_yaml:
                        params = _get_ros_params(raw_yaml, key)
                    return params

                switches = _get_ros_params_with_fallback("pb_navigation_switches")
                robot_description_runtime = _get_ros_params_with_fallback(
                    "robot_description_runtime"
                )

                candidate_robot_name = robot_description_runtime.get(
                    "robot_description_file",
                    switches.get("robot_description_file"),
                )
                if isinstance(candidate_robot_name, str):
                    candidate_robot_name = candidate_robot_name.strip()
                    if candidate_robot_name:
                        selected_robot_name = candidate_robot_name

        if not selected_robot_name:
            selected_robot_name = default_robot_name

        return [SetLaunchConfiguration("resolved_robot_name", selected_robot_name)]

    set_robot_name_cmd = OpaqueFunction(
        function=_set_robot_name_from_params,
        kwargs={
            "params_file": params_file,
            "namespace": namespace,
            "robot_name": robot_name,
        },
    )

    # Delay odin driver to avoid CPU contention during composable node loading.
    # The driver floods HIGHFREQ_ODOM data on connect, starving the container.
    start_odin_driver_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="odin_ros_driver",
                executable="host_sdk_sample",
                name="host_sdk_sample",
                output="screen",
                namespace=namespace,
                parameters=[{"config_file": odin_config_file}],
                condition=IfCondition(use_odin_driver),
            )
        ],
    )

    # Robot marker publisher: publishes an arrow marker to represent robot position and yaw
    start_robot_marker_publisher_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_marker_publisher",
                executable="robot_marker_publisher_node",
                name="robot_marker_publisher",
                output="screen",
                namespace=namespace,
                parameters=[{
                    "odometry_topic": "odin1/odometry",
                    "marker_topic": "robot_marker",
                    "marker_color_r": 0.0,
                    "marker_color_g": 1.0,
                    "marker_color_b": 0.0,
                    "arrow_side": 0.45,
                    "robot_radius": 0.225,
                }],
            )
        ],
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

    # NOTE: executable name is "foxglove_bridge" per ros2 convention.
    # If it fails to start, verify with: ros2 pkg executables foxglove_bridge
    def _launch_foxglove(context, *, use_foxglove_arg, topics_file_arg, namespace_arg):
        if not _optional_bool(use_foxglove_arg.perform(context)):
            return []
        topics_path = Path(topics_file_arg.perform(context)).expanduser()
        topic_whitelist = []
        if topics_path.is_file():
            for line in topics_path.read_text().splitlines():
                line = line.strip()
                if line and not line.startswith("#"):
                    topic_whitelist.append(f"^{re.escape(line)}$")
        ns = (namespace_arg.perform(context) or "").strip()
        node_name = "foxglove_bridge"
        params = {"address": "0.0.0.0", "port": 8765}
        if topic_whitelist:
            params["topic_whitelist"] = topic_whitelist
        # Services exposed to foxglove
        params["service_whitelist"] = [
            "^/start_waypoints$",
            "^/clear_waypoints$",
            "^/navigate_to_pose/_action/cancel_goal$",
            "^/follow_waypoints/_action/cancel_goal$",
        ]
        return [
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name=node_name,
                output="screen",
                namespace=ns,
                parameters=[params],
            ),
        ]

    foxglove_cmd = OpaqueFunction(
        function=_launch_foxglove,
        kwargs={
            "use_foxglove_arg": use_foxglove,
            "topics_file_arg": LaunchConfiguration("foxglove_topics_file"),
            "namespace_arg": namespace,
        },
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
            "publish_static_map_tf": publish_static_map_tf,
            "terrain_registered_scan_topic": terrain_registered_scan_topic,
            "terrain_lidar_odometry_topic": terrain_lidar_odometry_topic,
            "sensor_scan_registered_scan_topic": sensor_scan_registered_scan_topic,
            "sensor_scan_lidar_odometry_topic": sensor_scan_lidar_odometry_topic,
        }.items(),
    )

    # Relocalization fallback node: only for Mode 2 (relocalization).
    # Allows manual initial pose via RViz to unblock Nav2 when odin1 relocalization
    # takes too long. Stops publishing when odin1 relocalization succeeds.
    def _add_relocalization_fallback(context, *, odin_mode_arg):
        odin_mode_value = (odin_mode_arg.perform(context) or "").strip()
        if odin_mode_value != "2":
            return []
        fallback_script = os.path.join(bringup_dir, "scripts", "relocalization_fallback_node.py")
        if not os.path.isfile(fallback_script):
            return []
        fallback_cmd = ["python3", fallback_script]
        namespace_value = (namespace.perform(context) or "").strip()
        if namespace_value:
            fallback_cmd.extend(["--ros-args", "-r", f"__ns:={namespace_value}"])
        return [
            ExecuteProcess(
                cmd=fallback_cmd,
                name="relocalization_fallback",
                output="screen",
            ),
        ]

    relocalization_fallback_cmd = OpaqueFunction(
        function=_add_relocalization_fallback,
        kwargs={"odin_mode_arg": LaunchConfiguration("odin_map_mode")},
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
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_foxglove_cmd)
    ld.add_action(declare_foxglove_topics_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_odin_config_file_cmd)
    ld.add_action(declare_use_odin_driver_cmd)
    ld.add_action(declare_terrain_registered_scan_topic_cmd)
    ld.add_action(declare_terrain_lidar_odometry_topic_cmd)
    ld.add_action(declare_sensor_scan_registered_scan_topic_cmd)
    ld.add_action(declare_sensor_scan_lidar_odometry_topic_cmd)
    ld.add_action(declare_odin_map_mode_cmd)
    ld.add_action(resolve_map_cmd)
    ld.add_action(SetLaunchConfiguration("resolved_robot_name", "gxu2026_sentry_robot"))
    ld.add_action(set_robot_name_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_odin_driver_node)
    ld.add_action(start_robot_marker_publisher_node)
    ld.add_action(bringup_cmd)
    ld.add_action(relocalization_fallback_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(foxglove_cmd)

    # CALIB_HELPER_MODE:
    #   off/false/0  disable the independent RViz map calibration helper
    #   always/all/nav/slam  start it regardless of Nav2 localization mode
    calib_mode = os.environ.get("CALIB_HELPER_MODE", "always").strip().lower()
    calib_enabled = calib_mode not in ("0", "false", "no", "off", "disable", "disabled")

    calib_script = os.environ.get("CALIB_HELPER_SCRIPT", "")
    if not calib_script:
        _ws_candidate = Path(bringup_dir)
        for _ in range(6):
            _ws_candidate = _ws_candidate.parent
            _candidate = _ws_candidate / "scripts" / "calib_point_helper.py"
            if _candidate.exists():
                calib_script = str(_candidate)
                break

    if calib_enabled and calib_script and os.path.isfile(calib_script):
        calib_args = ["python3", calib_script]
        if os.environ.get("CALIB_NO_CSV", "").strip() in ("1", "true", "yes"):
            calib_args.append("--no-csv")
        start_calib_helper = ExecuteProcess(
            cmd=calib_args,
            output="screen",
        )
        ld.add_action(start_calib_helper)

    return ld
