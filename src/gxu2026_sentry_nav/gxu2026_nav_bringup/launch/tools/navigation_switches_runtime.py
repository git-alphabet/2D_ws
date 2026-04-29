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
    default_style_file = "rmuc_01.xml"
    enable_rm_bt = False
    style_file = default_style_file
    rm_bt_executable = "rm_behavior_tree"
    processed_file = str(params_path)
    controller_plugin_name = None
    neupan_frame_name = None
    enable_obstacle_scan_value = "false"
    enable_mid360_costmap_additive_value = "false"
    enable_odin1_loam_reframe_value = "false"
    enable_scan_additive_value = "false"
    obstacle_scan_output_topic_value = "obstacle_scan"
    enable_gimbal_yaw_bridge_value = False
    terrain_registered_scan_topic_value = "registered_scan"
    terrain_lidar_odometry_topic_value = "lidar_odometry"
    sensor_scan_registered_scan_topic_value = "registered_scan"
    sensor_scan_lidar_odometry_topic_value = "lidar_odometry"
    nav2_tf_warmup_target_frame_value = (
        nav2_tf_warmup_target_frame.perform(context) or "odom"
    ).strip() or "odom"
    nav2_tf_warmup_source_frame_value = (
        nav2_tf_warmup_source_frame.perform(context) or "gimbal_yaw_fake"
    ).strip() or "gimbal_yaw_fake"
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

        def _set_mid360_local_expected_update_rate(rate_value):
            def _resolve_local_params(container):
                if not isinstance(container, dict):
                    return None

                # 兼容两种结构：
                # 1) local_costmap.ros__parameters
                # 2) local_costmap.local_costmap.ros__parameters（当前 nav2_params 结构）
                candidates = [
                    ["local_costmap", "ros__parameters"],
                    ["local_costmap", "local_costmap", "ros__parameters"],
                ]
                for path in candidates:
                    current = container
                    valid = True
                    for key in path:
                        if not isinstance(current, dict):
                            valid = False
                            break
                        current = current.get(key)
                    if valid and isinstance(current, dict):
                        return current
                return None

            local_params = _resolve_local_params(target_data)
            if local_params is None and target_data is not raw_yaml:
                local_params = _resolve_local_params(raw_yaml)
            if local_params is None:
                return False

            voxel_layer = local_params.get("intensity_voxel_layer")
            if not isinstance(voxel_layer, dict):
                return False

            mid360_source = voxel_layer.get("terrain_map_mid360")
            if not isinstance(mid360_source, dict):
                return False

            current_value = mid360_source.get("expected_update_rate")
            if current_value == rate_value:
                return False

            mid360_source["expected_update_rate"] = rate_value
            return True

        def _set_costmap_observation_source_enabled(
            costmap_name, observation_layer_name, source_name, enabled
        ):
            def _resolve_costmap_params(container):
                if not isinstance(container, dict):
                    return None

                candidates = [
                    [costmap_name, "ros__parameters"],
                    [costmap_name, costmap_name, "ros__parameters"],
                ]
                for path in candidates:
                    current = container
                    valid = True
                    for key in path:
                        if not isinstance(current, dict):
                            valid = False
                            break
                        current = current.get(key)
                    if valid and isinstance(current, dict):
                        return current
                return None

            def _apply(container):
                costmap_params = _resolve_costmap_params(container)
                if costmap_params is None:
                    return False

                observation_layer = costmap_params.get(observation_layer_name)
                if not isinstance(observation_layer, dict):
                    return False

                changed = False
                raw_sources = observation_layer.get("observation_sources")
                if isinstance(raw_sources, str):
                    source_tokens = raw_sources.split()
                    filtered_tokens = [
                        token for token in source_tokens if token != source_name
                    ]
                    if enabled:
                        if source_name not in filtered_tokens:
                            filtered_tokens.append(source_name)
                    if filtered_tokens != source_tokens:
                        observation_layer["observation_sources"] = " ".join(
                            filtered_tokens
                        )
                        changed = True

                if enabled:
                    return changed

                if source_name in observation_layer:
                    observation_layer.pop(source_name, None)
                    changed = True

                return changed

            changed = _apply(target_data)
            if target_data is not raw_yaml:
                changed = _apply(raw_yaml) or changed
            return changed

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
        switch_override_required = False

        switches = _get_ros_params_with_fallback("pb_navigation_switches")
        mid360_runtime = _get_ros_params_with_fallback("mid360_runtime")
        obstacle_scan_runtime = _get_ros_params_with_fallback("obstacle_scan_runtime")
        scan_additive_runtime = _get_ros_params_with_fallback("scan_additive_runtime")
        loam_interface_runtime = _get_ros_params_with_fallback("loam_interface_runtime")
        terrain_analysis_runtime = _get_ros_params_with_fallback("terrain_analysis_runtime")
        sensor_scan_generation_runtime = _get_ros_params_with_fallback(
            "sensor_scan_generation_runtime"
        )
        gimbal_yaw_bridge_runtime = _get_ros_params_with_fallback(
            "gimbal_yaw_bridge_runtime"
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

        enable_rm_bt = bool(switches.get("enable_rm_behavior_tree", enable_rm_bt))

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

            # 容器工作区允许固定路径；宿主机不做用户目录硬编码。
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

        mid360_costmap_switch = _optional_bool(
            mid360_runtime.get(
                "enable_costmap_additive",
                switches.get("enable_mid360_costmap_additive"),
            )
        )
        mid360_costmap_in_slam_switch = _optional_bool(
            mid360_runtime.get("enable_costmap_additive_in_slam")
        )
        mid360_costmap_in_nav_switch = _optional_bool(
            mid360_runtime.get("enable_costmap_additive_in_nav")
        )
        if slam_enabled:
            # 建图阶段默认不引入 mid360，避免双源装配误差叠加。
            if mid360_costmap_in_slam_switch is not None:
                enable_mid360_costmap_additive_value = (
                    "true" if mid360_costmap_in_slam_switch else "false"
                )
            else:
                enable_mid360_costmap_additive_value = "false"
        else:
            if mid360_costmap_in_nav_switch is not None:
                enable_mid360_costmap_additive_value = (
                    "true" if mid360_costmap_in_nav_switch else "false"
                )
            elif mid360_costmap_switch is not None:
                enable_mid360_costmap_additive_value = (
                    "true" if mid360_costmap_switch else "false"
                )
            elif odometry_source == "odin1" and not sim_enabled:
                # Auto mode in odin1 reality: keep additive chain enabled,
                # and rely on actual mid360 source availability for outputs.
                enable_mid360_costmap_additive_value = "true"

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

        odin1_loam_reframe_switch = _optional_bool(
            loam_interface_runtime.get(
                "enable_odin1_loam_reframe",
                switches.get("enable_odin1_loam_reframe"),
            )
        )
        if odin1_loam_reframe_switch is not None:
            enable_odin1_loam_reframe_value = (
                "true" if odin1_loam_reframe_switch else "false"
            )

        odin1_external_axis_alignment_switch = _optional_bool(
            loam_interface_runtime.get(
                "enable_odin1_external_axis_alignment",
                switches.get("enable_odin1_external_axis_alignment"),
            )
        )
        odin1_external_axis_alignment_raw = loam_interface_runtime.get(
            "enable_odin1_external_axis_alignment",
            switches.get("enable_odin1_external_axis_alignment"),
        )
        odin1_external_axis_alignment_enabled = False
        if odin1_external_axis_alignment_switch is not None:
            odin1_external_axis_alignment_enabled = bool(
                odin1_external_axis_alignment_switch
            )
        elif isinstance(odin1_external_axis_alignment_raw, str):
            if odin1_external_axis_alignment_raw.strip().lower() == "auto":
                odin_internal_alignment_enabled = _detect_odin_internal_axis_alignment()
                odin1_external_axis_alignment_enabled = not odin_internal_alignment_enabled
                print(
                    "[navigation_launch] odin1 external axis alignment auto mode: "
                    f"internal={odin_internal_alignment_enabled}, "
                    f"external={odin1_external_axis_alignment_enabled}"
                )

        odin1_loam_reframe_enabled = (
            enable_odin1_loam_reframe_value == "true"
            and odometry_source == "odin1"
            and not sim_enabled
        )

        if odin1_loam_reframe_enabled:
            # odin1 走 loam 输出：点云/里程计都从 loam 统一出口进入下游链路。
            terrain_registered_scan_topic_value = "registered_scan"
            terrain_lidar_odometry_topic_value = "lidar_odometry"
            sensor_scan_registered_scan_topic_value = "registered_scan"
            sensor_scan_lidar_odometry_topic_value = "lidar_odometry"

            loam_params = target_data.setdefault("loam_interface", {}).setdefault(
                "ros__parameters", {}
            )
            desired_loam_params = {
                "state_estimation_topic": "odin1/odometry_highfreq",
                "registered_scan_topic": "odin1/cloud_slam",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "lidar_frame": "front_odin1",
                "input_odom_semantics": "odom_to_base",
                "input_cloud_semantics": "odom",
                "align_odin_axes": odin1_external_axis_alignment_enabled,
                "freeze_base_to_lidar_tf": False,
                "tf_lookup_timeout_sec": 0.2,
            }
            for key, value in desired_loam_params.items():
                if loam_params.get(key) != value:
                    loam_params[key] = value
                    switch_override_required = True

            sensor_scan_params = target_data.setdefault(
                "sensor_scan_generation", {}
            ).setdefault("ros__parameters", {})
            if sensor_scan_params.get("publish_base_tf") is not False:
                sensor_scan_params["publish_base_tf"] = False
                switch_override_required = True

        if not odin1_loam_reframe_enabled:
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
        gimbal_yaw_bridge_raw = gimbal_yaw_bridge_runtime.get("enabled")
        if gimbal_yaw_bridge_raw is None:
            gimbal_yaw_bridge_raw = gimbal_yaw_bridge_runtime.get(
                "enable_gimbal_yaw_bridge"
            )
        if gimbal_yaw_bridge_raw is not None:
            parsed_gimbal_bridge = _optional_bool(gimbal_yaw_bridge_raw)
            enable_gimbal_yaw_bridge_value = bool(parsed_gimbal_bridge)
        elif "enable_gimbal_yaw_bridge" in switches:
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
        rm_bt_executable = rm_bt_params.get("executable", rm_bt_executable)

        # 统一从 rm_behavior_tree 包中加载 RMUC 哨兵参数，避免在 nav2_params.yaml 里重复维护。
        try:
            rm_bt_share_dir = get_package_share_directory("rm_behavior_tree")
            rmuc_bt_params_file = os.path.join(
                rm_bt_share_dir, "config", "RMUC_2026", "rmuc_2026_params.yaml"
            )
            with open(rmuc_bt_params_file, "r", encoding="utf-8") as _f:
                _rmuc_params_yaml = yaml.safe_load(_f) or {}
            rmuc_rm_bt_params = _get_ros_params(_rmuc_params_yaml, "rm_behavior_tree")
            if isinstance(rmuc_rm_bt_params, dict) and rmuc_rm_bt_params:
                target_rm_bt = target_data.setdefault("rm_behavior_tree", {}).setdefault(
                    "ros__parameters", {}
                )
                for _k, _v in rmuc_rm_bt_params.items():
                    if target_rm_bt.get(_k) != _v:
                        target_rm_bt[_k] = copy.deepcopy(_v)
                        switch_override_required = True
        except Exception:
            pass
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
        # reality 目录加载 neupan.yaml，作为 FollowPath 的配置来源。
        # bringup nav2_params.yaml 中不应再有 FollowPath NeuPAN 块；即使残留也会被覆盖。
        if selected_plugin_key and selected_plugin_key.startswith("neupan_nav2_controller"):
            try:
                neupan_pkg_dir = get_package_share_directory("neupan_nav2_controller")
                neupan_yaml_path = os.path.join(
                    neupan_pkg_dir, "config", "reality", "neupan.yaml"
                )
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

        # scan_additive 依赖 mid360 costmap 链路，若开启 scan_additive 则强制开启 costmap additive。
        if enable_scan_additive_value == "true":
            enable_mid360_costmap_additive_value = "true"
            # scan_additive 需要 odin 原始 scan 输入，确保 pointcloud_to_laserscan 主链路开启。
            enable_obstacle_scan_value = "true"
            # 若仍输出到 obstacle_scan，会与 scan_additive_adapter 输出重名冲突。
            if obstacle_scan_output_topic_value == "obstacle_scan":
                obstacle_scan_output_topic_value = "scan_odin1"
        elif slam_enabled:
            # SLAM 单源建图时，必须把主 scan 直接发布到 obstacle_scan 给 slam_toolbox。
            obstacle_scan_output_topic_value = "obstacle_scan"

        # mid360 costmap 开关关闭时抑制 stale 告警；开启时恢复告警能力。
        mid360_expected_rate = 0.30 if enable_mid360_costmap_additive_value == "true" else 0.0
        if _set_mid360_local_expected_update_rate(mid360_expected_rate):
            switch_override_required = True
            override_required = True
        if _set_costmap_observation_source_enabled(
            "local_costmap",
            "intensity_voxel_layer",
            "terrain_map_mid360",
            enable_mid360_costmap_additive_value == "true",
        ):
            switch_override_required = True
            override_required = True
        if _set_costmap_observation_source_enabled(
            "global_costmap",
            "intensity_voxel_layer",
            "terrain_map_ext_mid360",
            enable_mid360_costmap_additive_value == "true",
        ):
            switch_override_required = True
            override_required = True

        # odin1 实车导航（非 slam）需要等待 map 链路就绪，
        # 否则会在重定位成功前提前激活 Nav2，导致持续报 map 帧不存在。
        if odometry_source == "odin1" and not sim_enabled and not slam_enabled:
            if nav2_tf_warmup_target_frame_value == "odom":
                nav2_tf_warmup_target_frame_value = "map"
            if nav2_tf_warmup_source_frame_value == "gimbal_yaw_fake":
                nav2_tf_warmup_source_frame_value = "gimbal_yaw_fake"

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
                f"enable_mid360_costmap_additive={enable_mid360_costmap_additive_value} "
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

    style_path = _resolve_bt_style_path(style_file) if enable_rm_bt else style_file
    return [
        SetLaunchConfiguration(
            "enable_rm_behavior_tree", "true" if enable_rm_bt else "false"
        ),
        SetLaunchConfiguration("rm_behavior_tree_executable", rm_bt_executable),
        SetLaunchConfiguration("rm_behavior_tree_style_path", style_path),
        SetLaunchConfiguration("processed_params_file", processed_file),
        SetLaunchConfiguration("enable_obstacle_scan", enable_obstacle_scan_value),
        SetLaunchConfiguration(
            "enable_mid360_costmap_additive",
            enable_mid360_costmap_additive_value,
        ),
        SetLaunchConfiguration(
            "enable_odin1_loam_reframe",
            enable_odin1_loam_reframe_value,
        ),
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
