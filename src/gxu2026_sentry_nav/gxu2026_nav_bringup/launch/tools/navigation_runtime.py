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

from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def _build_lifecycle_manager_node(
    *, use_sim_time, autostart, log_level, lifecycle_nodes, configured_params, condition=None
):
    return Node(
        condition=condition,
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )


def build_navigation_runtime_actions(
    *,
    bringup_dir,
    namespace,
    use_sim_time,
    autostart,
    use_composition,
    container_name_full,
    use_respawn,
    log_level,
    configured_params,
    terrain_registered_scan_topic,
    terrain_lidar_odometry_topic,
    sensor_scan_registered_scan_topic,
    sensor_scan_lidar_odometry_topic,
    enable_obstacle_scan,
    enable_scan_additive,
    enable_terrain_analysis,
    obstacle_scan_output_topic,
    nav2_tf_warmup_enabled,
    nav2_tf_warmup_target_frame,
    nav2_tf_warmup_source_frame,
    nav2_tf_warmup_timeout_sec,
    nav2_tf_warmup_check_hz,
    lifecycle_nodes,
):
    terrain_analysis_condition = IfCondition(enable_terrain_analysis)

    # terrain_analysis: single-source odin1 path
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
            ("/registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
            ("/lidar_odometry", terrain_lidar_odometry_topic),
        ],
        condition=terrain_analysis_condition,
    )

    # terrain_analysis_ext: single-source odin1 path
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
            ("/registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
            ("/lidar_odometry", terrain_lidar_odometry_topic),
        ],
        condition=terrain_analysis_condition,
    )

    # pointcloud_to_laserscan: terrain analysis enabled
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
            ("cloud_in", "terrain_map"),
            ("scan", "obstacle_scan"),
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    enable_obstacle_scan,
                    "' == 'true') and ('",
                    enable_terrain_analysis,
                    "' == 'true')",
                ]
            )
        ),
    )

    # pointcloud_to_laserscan: terrain analysis disabled (2D mode)
    start_pointcloud_to_laserscan_no_terrain_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", terrain_registered_scan_topic),
            ("scan", "obstacle_scan"),
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    enable_obstacle_scan,
                    "' == 'true') and ('",
                    enable_terrain_analysis,
                    "' != 'true')",
                ]
            )
        ),
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
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
                    ("/registered_scan", sensor_scan_registered_scan_topic),
                    ("lidar_odometry", sensor_scan_lidar_odometry_topic),
                    ("/lidar_odometry", sensor_scan_lidar_odometry_topic),
                ],
            ),
            start_pointcloud_to_laserscan_cmd,
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
                    ("cmd_vel", "cmd_vel_controller"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=[configured_params],
                remappings=[
                    ("registered_scan", sensor_scan_registered_scan_topic),
                    ("/registered_scan", sensor_scan_registered_scan_topic),
                    ("lidar_odometry", sensor_scan_lidar_odometry_topic),
                    ("/lidar_odometry", sensor_scan_lidar_odometry_topic),
                ],
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
                    ("cmd_vel", "cmd_vel_controller"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
            ),
        ],
    )

    # Lifecycle manager with autostart: automatically activates all managed
    # nodes after the container and odin driver have had time to initialize.
    # The odin driver starts at t=5s and needs additional time to connect
    # and publish TF (odom->base_footprint) before local costmap can activate.
    start_lifecycle_manager_cmd = TimerAction(
        period=12.0,  # 5s odin driver delay + 7s driver init margin
        actions=[
            _build_lifecycle_manager_node(
                use_sim_time=use_sim_time,
                autostart=autostart,
                log_level=log_level,
                lifecycle_nodes=lifecycle_nodes,
                configured_params=configured_params,
            )
        ],
    )

    return [
        start_terrain_analysis_cmd,
        start_terrain_analysis_ext_cmd,
        start_pointcloud_to_laserscan_cmd,
        start_pointcloud_to_laserscan_no_terrain_cmd,
        load_nodes,
        load_composable_nodes,
        start_lifecycle_manager_cmd,
    ]
