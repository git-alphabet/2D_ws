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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
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
    mid360_driver_dir = get_package_share_directory("mid360_driver")
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
    odin_config_file = LaunchConfiguration("odin_config_file")
    mid360_config_file = LaunchConfiguration("mid360_config_file")
    use_mid360_driver = LaunchConfiguration("use_mid360_driver")
    terrain_registered_scan_topic = LaunchConfiguration("terrain_registered_scan_topic")
    terrain_lidar_odometry_topic = LaunchConfiguration("terrain_lidar_odometry_topic")
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

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description=(
            "Robot xmacro basename override. Empty means reading "
            "pb_navigation_switches.robot_description_file from params_file."
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

    declare_odin_config_file_cmd = DeclareLaunchArgument(
        "odin_config_file",
        default_value=os.path.join(odin_driver_dir, "config", "control_command.yaml"),
        description="Full path to odin_ros_driver control config file",
    )

    declare_mid360_config_file_cmd = DeclareLaunchArgument(
        "mid360_config_file",
        default_value=os.path.join(mid360_driver_dir, "config", "param.yaml"),
        description="Full path to mid360_driver config file",
    )

    declare_use_mid360_driver_cmd = DeclareLaunchArgument(
        "use_mid360_driver",
        default_value="True",
        description="Whether to start mid360_driver in reality entry launch",
    )

    declare_terrain_registered_scan_topic_cmd = DeclareLaunchArgument(
        "terrain_registered_scan_topic",
        default_value="",
        description="Optional override for terrain point cloud input topic",
    )

    declare_terrain_lidar_odometry_topic_cmd = DeclareLaunchArgument(
        "terrain_lidar_odometry_topic",
        default_value="",
        description="Optional override for terrain odometry input topic",
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

    # Mode0: 需要我们发 static identity map->odom TF
    # Mode1/2: odin 自己发 map->odom TF，我们不能再发（否则冲突）
    from launch.substitutions import PythonExpression
    publish_static_map_tf = PythonExpression(
        ["'True' if '", LaunchConfiguration("odin_map_mode"), "' == '0' else 'False'"]
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

    def _set_robot_name_from_params(context, *, params_file, namespace, robot_name):
        default_robot_name = "pb2025_sentry_robot"
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

                switches = _get_ros_params(target_data, "pb_navigation_switches")
                if not switches and target_data is not raw_yaml:
                    switches = _get_ros_params(raw_yaml, "pb_navigation_switches")

                candidate_robot_name = switches.get("robot_description_file")
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

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "robot_name": LaunchConfiguration("resolved_robot_name"),
        }.items(),
    )

    start_odin_driver_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        namespace=namespace,
        parameters=[{"config_file": odin_config_file}],
    )

    start_mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        namespace=namespace,
        parameters=[mid360_config_file, configured_params],
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
            "publish_static_map_tf": publish_static_map_tf,
            "terrain_registered_scan_topic": terrain_registered_scan_topic,
            "terrain_lidar_odometry_topic": terrain_lidar_odometry_topic,
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
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_odin_config_file_cmd)
    ld.add_action(declare_mid360_config_file_cmd)
    ld.add_action(declare_use_mid360_driver_cmd)
    ld.add_action(declare_terrain_registered_scan_topic_cmd)
    ld.add_action(declare_terrain_lidar_odometry_topic_cmd)
    ld.add_action(declare_odin_map_mode_cmd)
    ld.add_action(SetLaunchConfiguration("resolved_robot_name", "pb2025_sentry_robot"))
    ld.add_action(set_robot_name_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_odin_driver_node)
    ld.add_action(start_mid360_driver_node)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
