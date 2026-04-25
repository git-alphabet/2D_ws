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
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
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
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    def _resolve_single_map_file(context, *, map_arg, slam_arg, map_dir):
        slam_value = (slam_arg.perform(context) or "").strip().lower()
        if slam_value in {"true", "1", "yes", "on"}:
            return []

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
            "slam_arg": slam,
            "map_dir": os.path.join(bringup_dir, "map", "simulation"),
        },
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will enable mapping mode and publish static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmuc_2025",
        description="Select world: 'rmul_2024' or 'rmuc_2024' or 'rmul_2025' or 'rmuc_2025' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description=(
            "Full path to map file to load. Empty means auto-detect exactly one yaml "
            "under map/simulation"
        ),
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
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

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
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
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(resolve_map_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    # 标定辅助节点（仅 SLAM 模式）
    calib_script = os.environ.get("CALIB_HELPER_SCRIPT", "")
    if not calib_script:
        _ws_candidate = Path(bringup_dir)
        for _ in range(6):
            _ws_candidate = _ws_candidate.parent
            _candidate = _ws_candidate / "scripts" / "calib_point_helper.py"
            if _candidate.exists():
                calib_script = str(_candidate)
                break
    if calib_script and os.path.isfile(calib_script):
        calib_args = ["python3", calib_script]
        if os.environ.get("CALIB_NO_CSV", "").strip().lower() in {"1", "true", "yes"}:
            calib_args.append("--no-csv")
        start_calib_helper = ExecuteProcess(
            cmd=calib_args,
            output="screen",
            condition=IfCondition(slam),
        )
        ld.add_action(start_calib_helper)

    return ld
