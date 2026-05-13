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
import sys
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

LAUNCH_DIR = Path(__file__).resolve().parent
TOOLS_DIR = LAUNCH_DIR / "tools"
for search_path in (LAUNCH_DIR, TOOLS_DIR):
    if str(search_path) not in sys.path:
        sys.path.insert(0, str(search_path))

try:
    from navigation_switches import build_set_switches_cmd
except ImportError:
    from navigation_switches_runtime import build_set_switches_cmd
from navigation_runtime import build_navigation_runtime_actions


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
    enable_fake_vel_transform_tf = LaunchConfiguration("enable_fake_vel_transform_tf")
    nav2_tf_warmup_enabled = LaunchConfiguration("nav2_tf_warmup_enabled")
    nav2_tf_warmup_target_frame = LaunchConfiguration("nav2_tf_warmup_target_frame")
    nav2_tf_warmup_source_frame = LaunchConfiguration("nav2_tf_warmup_source_frame")
    nav2_tf_warmup_timeout_sec = LaunchConfiguration("nav2_tf_warmup_timeout_sec")
    nav2_tf_warmup_check_hz = LaunchConfiguration("nav2_tf_warmup_check_hz")


    enable_gimbal_yaw_bridge = LaunchConfiguration("enable_gimbal_yaw_bridge")
    enable_rm_behavior_tree = LaunchConfiguration("enable_rm_behavior_tree")
    rm_behavior_tree_executable = LaunchConfiguration("rm_behavior_tree_executable")
    rm_behavior_tree_style_path = LaunchConfiguration("rm_behavior_tree_style_path")

    lifecycle_nodes = [
        "controller_server",
        "nav2_params.yaml
+1
-1
￼
￼
￼
23:13
￼
smoother_server",
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
            bringup_dir, "config", "reality", "point_lio_obstacle_only.yaml"
        ),
        description="Full path to point_lio config file for mid360 obstacle-only supplement chain",
    )

    declare_enable_fake_vel_transform_tf_cmd = DeclareLaunchArgument(
        "enable_fake_vel_transform_tf",
        default_value="True",
        description="Whether fake_vel_transform publishes gimbal_yaw->gimbal_yaw_fake TF. Set False during bag replay.",
    )

    declare_enable_odin1_loam_reframe_cmd = DeclareLaunchArgument(
        "enable_odin1_loam_reframe",
        default_value="false",
        description="Enable odin1->loam odometry reframing to output odom/lidar_frame semantics",
    )

    declare_nav2_tf_warmup_enabled_cmd = DeclareLaunchArgument(
        "nav2_tf_warmup_enabled",
        default_value="True",
        description="Wait for TF chain before starting nav2 lifecycle manager",
    )

    declare_nav2_tf_warmup_target_frame_cmd = DeclareLaunchArgument(
        "nav2_tf_warmup_target_frame",
        default_value="odom",
        description="TF warmup target frame",
    )

    declare_nav2_tf_warmup_source_frame_cmd = DeclareLaunchArgument(
        "nav2_tf_warmup_source_frame",
        default_value="gimbal_yaw_fake",
        description="TF warmup source frame",
    )

    declare_nav2_tf_warmup_timeout_sec_cmd = DeclareLaunchArgument(
        "nav2_tf_warmup_timeout_sec",
        default_value="25.0",
        description="TF warmup max wait seconds; <=0 disables timeout",
    )

    declare_nav2_tf_warmup_check_hz_cmd = DeclareLaunchArgument(
        "nav2_tf_warmup_check_hz",
        default_value="20.0",
        description="TF warmup polling frequency",
    )

    enable_obstacle_scan = LaunchConfiguration("enable_obstacle_scan")
    enable_mid360_costmap_additive = LaunchConfiguration("enable_mid360_costmap_additive")
    enable_odin1_loam_reframe = LaunchConfiguration("enable_odin1_loam_reframe")
    enable_scan_additive = LaunchConfiguration("enable_scan_additive")
    obstacle_scan_output_topic = LaunchConfiguration("obstacle_scan_output_topic")

    set_switches_cmd = build_set_switches_cmd(
        params_file=params_file,
        namespace=namespace,
        slam=slam,
        use_sim_time=use_sim_time,
        terrain_registered_scan_topic=terrain_registered_scan_topic,
        terrain_lidar_odometry_topic=terrain_lidar_odometry_topic,
        sensor_scan_registered_scan_topic=sensor_scan_registered_scan_topic,
        sensor_scan_lidar_odometry_topic=sensor_scan_lidar_odometry_topic,
        nav2_tf_warmup_target_frame=nav2_tf_warmup_target_frame,
        nav2_tf_warmup_source_frame=nav2_tf_warmup_source_frame,
        nav2_tf_warmup_timeout_sec=nav2_tf_warmup_timeout_sec,
    )

    def _build_runtime_actions(context):
        resolved_root_key = (namespace.perform(context) or "").lstrip("/")
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=processed_params_file.perform(context),
                root_key=resolved_root_key,
                param_rewrites=param_substitutions,
                convert_types=True,
            ),
            allow_substs=True,
        )

        return build_navigation_runtime_actions(
            bringup_dir=bringup_dir,
            namespace=namespace,
            use_sim_time=use_sim_time,
            autostart=autostart,
            use_composition=use_composition,
            container_name_full=container_name_full,
            use_respawn=use_respawn,
            log_level=log_level,
            configured_params=configured_params,
            point_lio_config_file=point_lio_config_file,
            terrain_registered_scan_topic=terrain_registered_scan_topic,
            terrain_lidar_odometry_topic=terrain_lidar_odometry_topic,
            sensor_scan_registered_scan_topic=sensor_scan_registered_scan_topic,
            sensor_scan_lidar_odometry_topic=sensor_scan_lidar_odometry_topic,
            enable_gimbal_yaw_bridge=enable_gimbal_yaw_bridge,
            enable_rm_behavior_tree=enable_rm_behavior_tree,
            rm_behavior_tree_executable=rm_behavior_tree_executable,
            rm_behavior_tree_style_path=rm_behavior_tree_style_path,
            enable_obstacle_scan=enable_obstacle_scan,
            enable_mid360_costmap_additive=enable_mid360_costmap_additive,
            enable_odin1_loam_reframe=enable_odin1_loam_reframe,
            enable_scan_additive=enable_scan_additive,
            obstacle_scan_output_topic=obstacle_scan_output_topic,
            enable_fake_vel_transform_tf=enable_fake_vel_transform_tf,
            nav2_tf_warmup_enabled=nav2_tf_warmup_enabled,
            nav2_tf_warmup_target_frame=nav2_tf_warmup_target_frame,
            nav2_tf_warmup_source_frame=nav2_tf_warmup_source_frame,
            nav2_tf_warmup_timeout_sec=nav2_tf_warmup_timeout_sec,
            nav2_tf_warmup_check_hz=nav2_tf_warmup_check_hz,
            lifecycle_nodes=lifecycle_nodes,
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
    ld.add_action(declare_enable_fake_vel_transform_tf_cmd)
    ld.add_action(declare_enable_odin1_loam_reframe_cmd)
    ld.add_action(declare_nav2_tf_warmup_enabled_cmd)
    ld.add_action(declare_nav2_tf_warmup_target_frame_cmd)
    ld.add_action(declare_nav2_tf_warmup_source_frame_cmd)
    ld.add_action(declare_nav2_tf_warmup_timeout_sec_cmd)
    ld.add_action(declare_nav2_tf_warmup_check_hz_cmd)
    # processed params defaults to original params file
    ld.add_action(SetLaunchConfiguration("processed_params_file", params_file))
    ld.add_action(SetLaunchConfiguration("enable_obstacle_scan", "false"))
    ld.add_action(SetLaunchConfiguration("rm_behavior_tree_executable", "rm_behavior_tree"))
    ld.add_action(SetLaunchConfiguration("enable_mid360_costmap_additive", "false"))
    ld.add_action(SetLaunchConfiguration("enable_odin1_loam_reframe", "false"))
    ld.add_action(SetLaunchConfiguration("enable_scan_additive", "false"))
    ld.add_action(SetLaunchConfiguration("obstacle_scan_output_topic", "obstacle_scan"))
    ld.add_action(SetLaunchConfiguration("terrain_registered_scan_topic", ""))
    ld.add_action(SetLaunchConfiguration("terrain_lidar_odometry_topic", ""))
    ld.add_action(SetLaunchConfiguration("sensor_scan_registered_scan_topic", ""))
    ld.add_action(SetLaunchConfiguration("sensor_scan_lidar_odometry_topic", ""))
    # Set switches before starting nodes
    ld.add_action(set_switches_cmd)
    ld.add_action(OpaqueFunction(function=lambda context: _build_runtime_actions(context)))

    return ld
