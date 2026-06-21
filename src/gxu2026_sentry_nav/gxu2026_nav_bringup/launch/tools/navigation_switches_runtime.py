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
import sys
import tempfile
from pathlib import Path

import yaml  # type: ignore

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch.actions import OpaqueFunction, SetLaunchConfiguration


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
    nav2_tf_warmup_target_frame,
    nav2_tf_warmup_source_frame,
    nav2_tf_warmup_timeout_sec,
):
    params_path = Path(params_file.perform(context)).expanduser()
    ns_value = namespace.perform(context)
    processed_file = str(params_path)
    controller_plugin_name = None
    enable_obstacle_scan_value = "false"
    enable_scan_additive_value = "false"
    obstacle_scan_output_topic_value = "obstacle_scan"
    terrain_registered_scan_topic_value = "registered_scan"
    terrain_lidar_odometry_topic_value = "lidar_odometry"
    sensor_scan_registered_scan_topic_value = "registered_scan"
    sensor_scan_lidar_odometry_topic_value = "lidar_odometry"
    nav2_tf_warmup_target_frame_value = (
        nav2_tf_warmup_target_frame.perform(context) or "odom"
    ).strip() or "odom"
    nav2_tf_warmup_source_frame_value = (
        nav2_tf_warmup_source_frame.perform(context) or "base_footprint"
    ).strip() or "base_footprint"
    nav2_tf_warmup_timeout_sec_value = (
        nav2_tf_warmup_timeout_sec.perform(context) or "25.0"
    ).strip() or "25.0"

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

        def _get_ros_params_with_fallback(key):
            params = _get_ros_params(target_data, key)
            if not params and target_data is not raw_yaml:
                params = _get_ros_params(raw_yaml, key)
            return params

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
            """Normalize width/height types (20 vs 20.0) in costmap params."""
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
        switch_override_required = False

        switches = _get_ros_params_with_fallback("pb_navigation_switches")
        obstacle_scan_runtime = _get_ros_params_with_fallback("obstacle_scan_runtime")
        scan_additive_runtime = _get_ros_params_with_fallback("scan_additive_runtime")
        terrain_analysis_runtime = _get_ros_params_with_fallback("terrain_analysis_runtime")
        enable_terrain_analysis_value = True  # default: enabled
        sensor_scan_generation_runtime = _get_ros_params_with_fallback(
            "sensor_scan_generation_runtime"
        )

        def _read_topic_name(runtime_params, runtime_key, legacy_switch_key):
            raw_value = runtime_params.get(runtime_key)
            if raw_value is None:
                raw_value = switches.get(legacy_switch_key)
            if isinstance(raw_value, str):
                topic_name = raw_value.strip()
                if topic_name:
                    return topic_name
            return ""

        switches_terrain_registered_scan_topic = _read_topic_name(
            terrain_analysis_runtime,
            "registered_scan_topic",
            "terrain_registered_scan_topic",
        )
        switches_terrain_lidar_odometry_topic = _read_topic_name(
            terrain_analysis_runtime,
            "lidar_odometry_topic",
            "terrain_lidar_odometry_topic",
        )
        switches_sensor_scan_registered_scan_topic = _read_topic_name(
            sensor_scan_generation_runtime,
            "registered_scan_topic",
            "sensor_scan_registered_scan_topic",
        )
        switches_sensor_scan_lidar_odometry_topic = _read_topic_name(
            sensor_scan_generation_runtime,
            "lidar_odometry_topic",
            "sensor_scan_lidar_odometry_topic",
        )

        odometry_source = switches.get("odometry_source")
        if isinstance(odometry_source, str):
            odometry_source = odometry_source.strip().lower()
        else:
            odometry_source = ""

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

        def _optional_nonnegative_float(raw_value):
            if isinstance(raw_value, bool):
                return None
            if isinstance(raw_value, (int, float)):
                value = float(raw_value)
            elif isinstance(raw_value, str):
                stripped = raw_value.strip()
                if not stripped:
                    return None
                try:
                    value = float(stripped)
                except ValueError:
                    return None
            else:
                return None
            if not math.isfinite(value) or value < 0.0:
                return None
            return value

        def _detect_odin_internal_axis_alignment():
            markers = (
                "align_odin_vector_to_vehicle",
                "corrected_x = -raw_y",
                "x'=-y, y'=x",
            )
            candidates = []

            env_src_dir = os.environ.get("ODIN_ROS_DRIVER_SOURCE_DIR", "").strip()
            if env_src_dir:
                env_root = Path(env_src_dir).expanduser()
                candidates.extend(
                    [
                        env_root / "include/host_sdk_sample.h",
                        env_root / "src/host_sdk_sample.cpp",
                    ]
                )

            # Container workspace allows fixed path; host does not hardcode user dir.
            candidates.extend(
                [
                    Path("/ws/src/odin_ros_driver/include/host_sdk_sample.h"),
                    Path("/ws/src/odin_ros_driver/src/host_sdk_sample.cpp"),
                ]
            )

            workspace_hints = []
            for env_name in ("WS_DIR", "COLCON_WORKSPACE", "PWD"):
                raw = os.environ.get(env_name, "").strip()
                if raw:
                    workspace_hints.append(Path(raw).expanduser())

            workspace_hints.append(Path.cwd())
            workspace_hints.extend(params_path.parents)

            seen_roots = set()
            for hint in workspace_hints:
                try:
                    hint_path = hint.resolve()
                except Exception:
                    hint_path = hint

                for root in [hint_path] + list(hint_path.parents):
                    key = str(root)
                    if key in seen_roots:
                        continue
                    seen_roots.add(key)

                    odin_root = root / "src/odin_ros_driver"
                    if odin_root.is_dir():
                        candidates.extend(
                            [
                                odin_root / "include/host_sdk_sample.h",
                                odin_root / "src/host_sdk_sample.cpp",
                            ]
                        )

            try:
                odin_share_dir = Path(get_package_share_directory("odin_ros_driver"))
                odin_prefix_dir = odin_share_dir.parent.parent
                candidates.append(
                    odin_prefix_dir / "include/odin_ros_driver/host_sdk_sample.h"
                )
            except PackageNotFoundError:
                pass

            seen = set()
            for file_path in candidates:
                key = str(file_path)
                if key in seen:
                    continue
                seen.add(key)
                if not file_path.is_file():
                    continue
                try:
                    content = file_path.read_text(errors="ignore")
                except Exception:
                    continue
                if any(marker in content for marker in markers):
                    return True

            return False

        terrain_analysis_switch = _optional_bool(
            terrain_analysis_runtime.get("enable_terrain_analysis")
        )
        if terrain_analysis_switch is not None:
            enable_terrain_analysis_value = terrain_analysis_switch

        obstacle_scan_switch = _optional_bool(
            obstacle_scan_runtime.get("enabled", switches.get("enable_obstacle_scan"))
        )
        if obstacle_scan_switch is not None:
            enable_obstacle_scan_value = "true" if obstacle_scan_switch else "false"

        scan_additive_switch = _optional_bool(
            scan_additive_runtime.get("enabled", switches.get("enable_scan_additive"))
        )
        scan_additive_in_slam_switch = _optional_bool(
            scan_additive_runtime.get("enabled_in_slam")
        )
        scan_additive_in_nav_switch = _optional_bool(
            scan_additive_runtime.get("enabled_in_nav")
        )
        if slam_enabled:
            if scan_additive_in_slam_switch is not None:
                enable_scan_additive_value = (
                    "true" if scan_additive_in_slam_switch else "false"
                )
            else:
                enable_scan_additive_value = "false"
        else:
            if scan_additive_in_nav_switch is not None:
                enable_scan_additive_value = (
                    "true" if scan_additive_in_nav_switch else "false"
                )
            elif scan_additive_switch is not None:
                enable_scan_additive_value = "true" if scan_additive_switch else "false"

        obstacle_scan_output_topic_switch = obstacle_scan_runtime.get(
            "output_scan_topic", switches.get("obstacle_scan_output_topic")
        )
        if isinstance(obstacle_scan_output_topic_switch, str):
            obstacle_scan_output_topic_switch = obstacle_scan_output_topic_switch.strip()
            if obstacle_scan_output_topic_switch:
                obstacle_scan_output_topic_value = obstacle_scan_output_topic_switch

        if switches_terrain_registered_scan_topic:
            terrain_registered_scan_topic_value = switches_terrain_registered_scan_topic
        elif odometry_source == "odin1" and not sim_enabled:
            terrain_registered_scan_topic_value = "odin1/cloud_slam"

        if switches_terrain_lidar_odometry_topic:
            terrain_lidar_odometry_topic_value = switches_terrain_lidar_odometry_topic
        elif odometry_source == "odin1" and not sim_enabled:
            terrain_lidar_odometry_topic_value = "odin1/odometry"

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
            sensor_scan_lidar_odometry_topic_value = "odin1/odometry"

        plugin_from_params = switches.get("controller_plugin")
        if isinstance(plugin_from_params, str):
            plugin_candidate = plugin_from_params.strip()
            if plugin_candidate:
                controller_plugin_name = plugin_candidate

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

        frame_override_paths = [
            ["bt_navigator", "ros__parameters", "robot_base_frame"],
            ["local_costmap", "local_costmap", "ros__parameters", "robot_base_frame"],
            ["global_costmap", "global_costmap", "ros__parameters", "robot_base_frame"],
            ["behavior_server", "ros__parameters", "robot_base_frame"],
        ]

        override_required = params_normalized or switch_override_required
        if selected_plugin_key and selected_plugin_key in available_profiles:
            target_profile = available_profiles[selected_plugin_key]
            plugin_field = (
                target_profile.get("plugin") if isinstance(target_profile, dict) else ""
            )
            current_profile = controller_server.get(active_plugin_slot)
            if current_profile != target_profile:
                controller_server[active_plugin_slot] = copy.deepcopy(target_profile)
                override_required = True

        if enable_scan_additive_value == "true":
            # scan_additive needs odin raw scan input, ensure pointcloud_to_laserscan main chain is on.
            enable_obstacle_scan_value = "true"
            # Avoid output name collision with scan_additive output.
            if obstacle_scan_output_topic_value == "obstacle_scan":
                obstacle_scan_output_topic_value = "scan_odin1"
        elif slam_enabled:
            # SLAM single-source mapping: publish main scan directly to obstacle_scan for slam_toolbox.
            obstacle_scan_output_topic_value = "obstacle_scan"

        # odin1 pure navigation usually needs map TF before Nav2 activation.
        if odometry_source == "odin1" and not sim_enabled and not slam_enabled:
            if nav2_tf_warmup_target_frame_value == "odom":
                nav2_tf_warmup_target_frame_value = "map"

        if override_required:
            local_costmap_params = _get_ros_params_with_fallback("local_costmap")
            local_sources = None
            if isinstance(local_costmap_params.get("local_costmap"), dict):
                local_sources = (
                    local_costmap_params["local_costmap"]
                    .get("ros__parameters", {})
                    .get("intensity_voxel_layer", {})
                    .get("observation_sources")
                )
            elif isinstance(local_costmap_params.get("intensity_voxel_layer"), dict):
                local_sources = (
                    local_costmap_params.get("intensity_voxel_layer", {}).get("observation_sources")
                )

            global_costmap_params = _get_ros_params_with_fallback("global_costmap")
            global_sources = None
            if isinstance(global_costmap_params.get("global_costmap"), dict):
                global_sources = (
                    global_costmap_params["global_costmap"]
                    .get("ros__parameters", {})
                    .get("intensity_voxel_layer", {})
                    .get("observation_sources")
                )
            elif isinstance(global_costmap_params.get("intensity_voxel_layer"), dict):
                global_sources = (
                    global_costmap_params.get("intensity_voxel_layer", {}).get("observation_sources")
                )

            with tempfile.NamedTemporaryFile(
                mode="w", delete=False, suffix=".yaml"
            ) as tmp_file:
                yaml.safe_dump(raw_yaml, tmp_file, default_flow_style=False)
                processed_file = tmp_file.name
            print(
                "[navigation_launch] processed params: "
                f"slam={slam_enabled} "
                f"local_sources={local_sources!r} "
                f"global_sources={global_sources!r} "
                f"file={processed_file}",
                file=sys.stderr,
            )

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

    return [
        SetLaunchConfiguration("processed_params_file", processed_file),
        SetLaunchConfiguration("enable_obstacle_scan", enable_obstacle_scan_value),
        SetLaunchConfiguration("enable_scan_additive", enable_scan_additive_value),
        SetLaunchConfiguration(
            "enable_terrain_analysis",
            "true" if enable_terrain_analysis_value else "false",
        ),
        SetLaunchConfiguration(
            "obstacle_scan_output_topic", obstacle_scan_output_topic_value
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
        SetLaunchConfiguration(
            "nav2_tf_warmup_target_frame",
            nav2_tf_warmup_target_frame_value,
        ),
        SetLaunchConfiguration(
            "nav2_tf_warmup_source_frame",
            nav2_tf_warmup_source_frame_value,
        ),
        SetLaunchConfiguration(
            "nav2_tf_warmup_timeout_sec",
            nav2_tf_warmup_timeout_sec_value,
        ),
    ]


def build_set_switches_cmd(
    *,
    params_file,
    namespace,
    slam,
    use_sim_time,
    terrain_registered_scan_topic,
    terrain_lidar_odometry_topic,
    sensor_scan_registered_scan_topic,
    sensor_scan_lidar_odometry_topic,
    nav2_tf_warmup_target_frame,
    nav2_tf_warmup_source_frame,
    nav2_tf_warmup_timeout_sec,
):
    return OpaqueFunction(
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
            "nav2_tf_warmup_target_frame": nav2_tf_warmup_target_frame,
            "nav2_tf_warmup_source_frame": nav2_tf_warmup_source_frame,
            "nav2_tf_warmup_timeout_sec": nav2_tf_warmup_timeout_sec,
        },
    )
