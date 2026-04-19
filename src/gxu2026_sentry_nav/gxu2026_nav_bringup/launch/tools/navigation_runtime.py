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

from launch.actions import ExecuteProcess, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def _build_lifecycle_manager_node(
    *, use_sim_time, autostart, log_level, lifecycle_nodes, condition=None
):
    return Node(
        condition=condition,
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
    point_lio_config_file,
    terrain_registered_scan_topic,
    terrain_lidar_odometry_topic,
    sensor_scan_registered_scan_topic,
    sensor_scan_lidar_odometry_topic,
    enable_gimbal_yaw_bridge,
    enable_rm_behavior_tree,
    rm_behavior_tree_style_path,
    enable_obstacle_scan,
    enable_mid360_costmap_additive,
    enable_odin1_loam_reframe,
    enable_scan_additive,
    obstacle_scan_output_topic,
    nav2_tf_warmup_enabled,
    nav2_tf_warmup_target_frame,
    nav2_tf_warmup_source_frame,
    nav2_tf_warmup_timeout_sec,
    nav2_tf_warmup_check_hz,
    lifecycle_nodes,
):
    enable_loam_interface = PythonExpression(
        [
            "('",
            enable_mid360_costmap_additive,
            "' == 'true') or ('",
            enable_odin1_loam_reframe,
            "' == 'true')",
        ]
    )

    scan_additive_condition = IfCondition(
        PythonExpression(
            [
                "('",
                enable_scan_additive,
                "' == 'true') and ('",
                enable_mid360_costmap_additive,
                "' == 'true')",
            ]
        )
    )

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
            ("cloud_registered", "mid360/cloud_registered"),
            ("aft_mapped_to_init", "mid360/point_lio_odometry"),
        ],
        condition=IfCondition(enable_mid360_costmap_additive),
    )

    start_loam_interface_mid360_cmd = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface_mid360",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("registered_scan", "mid360/registered_scan"),
            ("lidar_odometry", "mid360/lidar_odometry"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        condition=IfCondition(enable_mid360_costmap_additive),
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
            ("registered_scan", "mid360/registered_scan"),
            ("/registered_scan", "mid360/registered_scan"),
            ("lidar_odometry", "mid360/lidar_odometry"),
            ("/lidar_odometry", "mid360/lidar_odometry"),
            ("terrain_map", "terrain_map_mid360"),
            ("/terrain_map", "terrain_map_mid360"),
            ("terrain_map_ext", "terrain_map_ext_mid360"),
            ("reference_terrain_map_ext", "/terrain_map_ext"),
            # Keep the reference topic absolute path untouched for auto stamp sync.
            ("/terrain_map_ext", "/terrain_map_ext"),
        ],
        condition=IfCondition(enable_mid360_costmap_additive),
    )

    start_terrain_analysis_mid360_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis_mid360",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[
            ("registered_scan", "mid360/registered_scan"),
            ("/registered_scan", "mid360/registered_scan"),
            ("lidar_odometry", "mid360/lidar_odometry"),
            ("/lidar_odometry", "mid360/lidar_odometry"),
            ("terrain_map", "terrain_map_mid360"),
            ("reference_terrain_map", "/terrain_map"),
            # Keep the reference topic absolute path untouched for auto stamp sync.
            ("/terrain_map", "/terrain_map"),
        ],
        condition=IfCondition(enable_mid360_costmap_additive),
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
            ("cloud_in", "terrain_map_mid360"),
            ("scan", "scan_mid360"),
        ],
        condition=scan_additive_condition,
    )

    start_scan_additive_adapter_cmd = Node(
        package="scan_additive_adapter",
        executable="scan_additive_adapter_node",
        name="scan_additive_adapter",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        condition=scan_additive_condition,
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
            ("/registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
            ("/lidar_odometry", terrain_lidar_odometry_topic),
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
            ("/registered_scan", terrain_registered_scan_topic),
            ("lidar_odometry", terrain_lidar_odometry_topic),
            ("/lidar_odometry", terrain_lidar_odometry_topic),
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
                condition=IfCondition(enable_loam_interface),
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
                    ("/registered_scan", sensor_scan_registered_scan_topic),
                    ("lidar_odometry", sensor_scan_lidar_odometry_topic),
                    ("/lidar_odometry", sensor_scan_lidar_odometry_topic),
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
                remappings=[("cmd_vel", "cmd_vel_nav2_result")],
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
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),
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
                remappings=[("cmd_vel", "cmd_vel_nav2_result")],
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
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),
                ],
            ),
        ],
    )

    load_loam_composable_node = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_composition,
                    "' == 'True') and (('",
                    enable_mid360_costmap_additive,
                    "' == 'true') or ('",
                    enable_odin1_loam_reframe,
                    "' == 'true'))",
                ]
            )
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=[configured_params],
            )
        ],
    )

    wait_nav2_tf_warmup_cmd = ExecuteProcess(
        condition=IfCondition(nav2_tf_warmup_enabled),
        cmd=[
            "python3",
            os.path.join(bringup_dir, "launch", "nav2_tf_warmup_wait.py"),
            "--target-frame",
            nav2_tf_warmup_target_frame,
            "--source-frame",
            nav2_tf_warmup_source_frame,
            "--timeout-sec",
            nav2_tf_warmup_timeout_sec,
            "--check-hz",
            nav2_tf_warmup_check_hz,
            "--namespace",
            namespace,
            "--use-sim-time",
            use_sim_time,
        ],
        output="screen",
    )

    start_lifecycle_manager_cmd_direct = _build_lifecycle_manager_node(
        use_sim_time=use_sim_time,
        autostart=autostart,
        log_level=log_level,
        lifecycle_nodes=lifecycle_nodes,
        condition=UnlessCondition(nav2_tf_warmup_enabled),
    )

    start_lifecycle_manager_after_warmup_cmd = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_nav2_tf_warmup_cmd,
            on_exit=[
                _build_lifecycle_manager_node(
                    use_sim_time=use_sim_time,
                    autostart=autostart,
                    log_level=log_level,
                    lifecycle_nodes=lifecycle_nodes,
                )
            ],
        )
    )

    start_auto_aim_yaw_joint_state_bridge_cmd = Node(
        condition=IfCondition(enable_gimbal_yaw_bridge),
        package="gimbal_yaw_bridge",
        executable="auto_aim_yaw_joint_state_bridge",
        name="auto_aim_yaw_joint_state_bridge",
        output="screen",
        parameters=[configured_params],
    )

    return [
        start_auto_aim_yaw_joint_state_bridge_cmd,
        start_terrain_analysis_cmd,
        start_terrain_analysis_mid360_cmd,
        start_terrain_analysis_ext_cmd,
        start_point_lio_cmd,
        start_loam_interface_mid360_cmd,
        start_terrain_analysis_ext_mid360_cmd,
        start_pointcloud_to_laserscan_mid360_cmd,
        start_scan_additive_adapter_cmd,
        start_rm_behavior_tree_cmd,
        load_nodes,
        load_loam_composable_node,
        load_composable_nodes,
        load_pointcloud_to_laserscan_composable_cmd,
        wait_nav2_tf_warmup_cmd,
        start_lifecycle_manager_cmd_direct,
        start_lifecycle_manager_after_warmup_cmd,
    ]
