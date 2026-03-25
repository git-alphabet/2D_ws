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


# NOTE: This startup file is only used when the navigation module is standalone
# It is used to launch the robot state publisher and joint state publisher.
# But in a complete robot system, this part should be completed by an independent robot startup module

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def _resolve_robot_name_from_params(
    params_file_path: str, namespace_value: str, fallback_robot_name: str
) -> str:
    if not params_file_path:
        return fallback_robot_name

    try:
        with open(params_file_path, "r", encoding="utf-8") as file_obj:
            yaml_obj = yaml.safe_load(file_obj) or {}
    except Exception:
        return fallback_robot_name

    if not isinstance(yaml_obj, dict):
        return fallback_robot_name

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
        robot_name_value = ros_params.get("robot_name")
        if isinstance(robot_name_value, str) and robot_name_value.strip():
            return robot_name_value.strip()

    return fallback_robot_name


def launch_setup(context: LaunchContext):
    pkg_gxu2026_robot_description_dir = get_package_share_directory(
        "gxu2026_robot_description"
    )

    namespace_value = (context.launch_configurations.get("namespace") or "").strip()
    use_sim_time_value = (
        context.launch_configurations.get("use_sim_time") or "false"
    ).strip()
    fallback_robot_name = (
        context.launch_configurations.get("robot_name") or "simulation_robot"
    ).strip()
    params_file_value = (context.launch_configurations.get("params_file") or "").strip()

    resolved_robot_name = _resolve_robot_name_from_params(
        params_file_path=params_file_value,
        namespace_value=namespace_value,
        fallback_robot_name=fallback_robot_name,
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace_value),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_gxu2026_robot_description_dir,
                        "launch",
                        "robot_description_launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": namespace_value,
                    "use_sim_time": use_sim_time_value,
                    "robot_name": resolved_robot_name,
                    # Avoid inheriting the parent launch's `params_file` (nav2_params.yaml).
                    # robot_description_launch.py expects gxu2026_robot_description params.
                    "params_file": os.path.join(
                        pkg_gxu2026_robot_description_dir, "params", "robot_description.yaml"
                    ),
                    "use_rviz": "False",
                }.items(),
            ),
        ]
    )

    return [bringup_cmd_group]


def generate_launch_description():
    pkg_gxu2026_nav_bringup_dir = get_package_share_directory("gxu2026_nav_bringup")

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="simulation_robot",
            description="Fallback robot model name when not found in params_file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                pkg_gxu2026_nav_bringup_dir,
                "config",
                "reality",
                "nav2_params.yaml",
            ),
            description="Full path to gxu2026_nav_bringup nav2 params yaml (single source of robot_name)",
        )
    )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
