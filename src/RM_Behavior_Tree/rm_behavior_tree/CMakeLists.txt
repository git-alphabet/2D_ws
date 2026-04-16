cmake_minimum_required(VERSION 3.10)
project(rm_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(PACKAGE_INCLUDE_DEPENDS
    sp_msgs
    rm_decision_interfaces
    rclcpp
    std_msgs
    behaviortree_cpp
    behaviortree_ros2
    nav2_msgs
    nav_msgs
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)
ament_auto_add_executable(rm_behavior_tree_minimal
    src/rm_behavior_tree_minimal.cpp
)
ament_auto_add_executable(rm_behavior_tree_rmuc
    src/rm_behavior_tree_rmuc.cpp
)
# Controllers

# Decorators
ament_auto_add_library(rate_controller SHARED 
    plugins/rmul_2026/decorator/rate_controller.cpp
)
add_target_dependencies(rate_controller)

# Conditions
ament_auto_add_library(is_game_time SHARED 
    plugins/rmul_2026/condition/is_game_time.cpp
)
add_target_dependencies(is_game_time)

ament_auto_add_library(is_status_ok SHARED 
    plugins/rmul_2026/condition/is_status_ok.cpp
)
add_target_dependencies(is_status_ok)

ament_auto_add_library(is_detect_enemy SHARED 
    plugins/rmul_2026/condition/is_detect_enemy.cpp
)
add_target_dependencies(is_detect_enemy)

ament_auto_add_library(is_friend_ok SHARED 
    plugins/rmul_2026/condition/is_friend_ok.cpp
)
add_target_dependencies(is_friend_ok)


## is_detect_enemy skipped: depends on external package 'auto_aim_interfaces' not available in this workspace


ament_auto_add_library(is_hp_above SHARED 
    plugins/rmul_2026/condition/is_hp_above.cpp
)
add_target_dependencies(is_hp_above)

ament_auto_add_library(is_hp_below SHARED 
    plugins/rmul_2026/condition/is_hp_below.cpp
)
add_target_dependencies(is_hp_below)

ament_auto_add_library(is_hp_increasing SHARED
    plugins/rmul_2026/condition/is_hp_increasing.cpp
)
add_target_dependencies(is_hp_increasing)

ament_auto_add_library(is_dead SHARED 
    plugins/rmul_2026/condition/is_dead.cpp
)
add_target_dependencies(is_dead)

# ── RMUC 2026 LowHPRetreat 子树插件 ──────────────────────────────────────
# Conditions
ament_auto_add_library(rmuc_is_dead SHARED
    plugins/rmuc_2026/condition/is_dead.cpp
)
add_target_dependencies(rmuc_is_dead)

ament_auto_add_library(rmuc_is_detect_enemy SHARED
    plugins/rmuc_2026/condition/is_detect_enemy.cpp
)
add_target_dependencies(rmuc_is_detect_enemy)

# Actions
ament_auto_add_library(rmuc_robot_control SHARED
    plugins/rmuc_2026/action/robot_control.cpp
)
add_target_dependencies(rmuc_robot_control)

ament_auto_add_library(rmuc_nav_control_cmd SHARED
    plugins/rmuc_2026/action/nav_control_cmd.cpp
)
add_target_dependencies(rmuc_nav_control_cmd)

# ── RMUC 2026 镜像插件 (6个：Sub[Game/Robot/RFID/Position] + IsGameTime + IsHPBelow) ──
ament_auto_add_library(rmuc_sub_game_status SHARED
    plugins/rmuc_2026/action/sub_game_status.cpp)
add_target_dependencies(rmuc_sub_game_status)

ament_auto_add_library(rmuc_sub_robot_status SHARED
    plugins/rmuc_2026/action/sub_robot_status.cpp)
add_target_dependencies(rmuc_sub_robot_status)

ament_auto_add_library(rmuc_sub_rfid_status SHARED
    plugins/rmuc_2026/action/sub_rfid_status.cpp)
add_target_dependencies(rmuc_sub_rfid_status)

ament_auto_add_library(rmuc_sub_robot_position SHARED
    plugins/rmuc_2026/action/sub_robot_position.cpp)
add_target_dependencies(rmuc_sub_robot_position)

ament_auto_add_library(rmuc_sub_radar_tracks SHARED
    plugins/rmuc_2026/action/sub_radar_tracks.cpp)
add_target_dependencies(rmuc_sub_radar_tracks)

ament_auto_add_library(rmuc_is_game_time SHARED
    plugins/rmuc_2026/condition/is_game_time.cpp)
add_target_dependencies(rmuc_is_game_time)

ament_auto_add_library(rmuc_is_hp_below SHARED
    plugins/rmuc_2026/condition/is_hp_below.cpp)
add_target_dependencies(rmuc_is_hp_below)

# ── RMUC 2026 P0 新增 7 个裁判系统话题订阅插件 ──
ament_auto_add_library(rmuc_sub_sentry_decision_status SHARED
    plugins/rmuc_2026/action/sub_sentry_decision_status.cpp)
add_target_dependencies(rmuc_sub_sentry_decision_status)

ament_auto_add_library(rmuc_sub_robot_buff SHARED
    plugins/rmuc_2026/action/sub_robot_buff.cpp)
add_target_dependencies(rmuc_sub_robot_buff)

ament_auto_add_library(rmuc_sub_projectile_allowance SHARED
    plugins/rmuc_2026/action/sub_projectile_allowance.cpp)
add_target_dependencies(rmuc_sub_projectile_allowance)

ament_auto_add_library(rmuc_sub_field_status SHARED
    plugins/rmuc_2026/action/sub_field_status.cpp)
add_target_dependencies(rmuc_sub_field_status)

ament_auto_add_library(rmuc_sub_enemy_mark SHARED
    plugins/rmuc_2026/action/sub_enemy_mark.cpp)
add_target_dependencies(rmuc_sub_enemy_mark)

ament_auto_add_library(rmuc_sub_team_positions SHARED
    plugins/rmuc_2026/action/sub_team_positions.cpp)
add_target_dependencies(rmuc_sub_team_positions)

ament_auto_add_library(rmuc_sub_team_hp SHARED
    plugins/rmuc_2026/action/sub_team_hp.cpp)
add_target_dependencies(rmuc_sub_team_hp)

# ── RMUC 2026 感知/指令类 RMUC 特有插件 ──
ament_auto_add_library(rmuc_parse_sentry_blackboard SHARED
    plugins/rmuc_2026/action/parse_sentry_blackboard.cpp)
add_target_dependencies(rmuc_parse_sentry_blackboard)

ament_auto_add_library(rmuc_sentry_cmd_mux SHARED
    plugins/rmuc_2026/action/sentry_cmd_mux.cpp)
add_target_dependencies(rmuc_sentry_cmd_mux)

ament_auto_add_library(rmuc_decide_respawn_cmd SHARED
    plugins/rmuc_2026/action/decide_respawn_cmd.cpp)
add_target_dependencies(rmuc_decide_respawn_cmd)

ament_auto_add_library(rmuc_is_zone_card_detected SHARED
    plugins/rmuc_2026/condition/is_zone_card_detected.cpp)
add_target_dependencies(rmuc_is_zone_card_detected)

# ── RMUC 2026 决策/初始化动作插件 ──
ament_auto_add_library(rmuc_init_sentry_config SHARED
    plugins/rmuc_2026/action/init_sentry_config.cpp)
add_target_dependencies(rmuc_init_sentry_config)

ament_auto_add_library(rmuc_init_cmd_state SHARED
    plugins/rmuc_2026/action/init_cmd_state.cpp)
add_target_dependencies(rmuc_init_cmd_state)

ament_auto_add_library(rmuc_decide_posture SHARED
    plugins/rmuc_2026/action/decide_posture.cpp)
add_target_dependencies(rmuc_decide_posture)

ament_auto_add_library(rmuc_select_posture SHARED
    plugins/rmuc_2026/action/select_posture.cpp)
add_target_dependencies(rmuc_select_posture)

ament_auto_add_library(rmuc_posture_degradation_guard SHARED
    plugins/rmuc_2026/action/posture_degradation_guard.cpp)
add_target_dependencies(rmuc_posture_degradation_guard)

ament_auto_add_library(rmuc_decide_economy_cmd SHARED
    plugins/rmuc_2026/action/decide_economy_cmd.cpp)
add_target_dependencies(rmuc_decide_economy_cmd)

ament_auto_add_library(rmuc_load_calibration_csv SHARED
    plugins/rmuc_2026/action/load_calibration_csv.cpp)
add_target_dependencies(rmuc_load_calibration_csv)

# ── RMUC 2026 后勤/补给 ──
ament_auto_add_library(rmuc_hold_and_heal SHARED
    plugins/rmuc_2026/action/hold_and_heal.cpp)
add_target_dependencies(rmuc_hold_and_heal)

ament_auto_add_library(rmuc_hold_for_supply_ammo_tick SHARED
    plugins/rmuc_2026/action/hold_for_supply_ammo_tick.cpp)
add_target_dependencies(rmuc_hold_for_supply_ammo_tick)

ament_auto_add_library(rmuc_select_nearest_resupply_station SHARED
    plugins/rmuc_2026/action/select_nearest_resupply_station.cpp)
add_target_dependencies(rmuc_select_nearest_resupply_station)

# ── RMUC 2026 目标/巡逻 ──
ament_auto_add_library(rmuc_select_objective SHARED
    plugins/rmuc_2026/action/select_objective.cpp)
add_target_dependencies(rmuc_select_objective)

ament_auto_add_library(rmuc_hold_objective SHARED
    plugins/rmuc_2026/action/hold_objective.cpp)
add_target_dependencies(rmuc_hold_objective)

ament_auto_add_library(rmuc_objective_patrol SHARED
    plugins/rmuc_2026/action/objective_patrol.cpp)
add_target_dependencies(rmuc_objective_patrol)

# ── RMUC 2026 条件插件 ──
ament_auto_add_library(rmuc_is_ammo_below SHARED
    plugins/rmuc_2026/condition/is_ammo_below.cpp)
add_target_dependencies(rmuc_is_ammo_below)

ament_auto_add_library(rmuc_is_base_threatened SHARED
    plugins/rmuc_2026/condition/is_base_threatened.cpp)
add_target_dependencies(rmuc_is_base_threatened)

ament_auto_add_library(rmuc_is_combat_allowed SHARED
    plugins/rmuc_2026/condition/is_combat_allowed.cpp)
add_target_dependencies(rmuc_is_combat_allowed)

ament_auto_add_library(rmuc_is_at_goal SHARED
    plugins/rmuc_2026/condition/is_at_goal.cpp)
add_target_dependencies(rmuc_is_at_goal)

ament_auto_add_library(rmuc_is_nav_target_supply SHARED
    plugins/rmuc_2026/condition/is_nav_target_supply.cpp)
add_target_dependencies(rmuc_is_nav_target_supply)

# ── 结束 RMUC 2026 全部插件 ────────────────────────────────────────────────

ament_auto_add_library(is_recovery_needed SHARED 
    plugins/rmul_2026/condition/is_recovery_needed.cpp
)
add_target_dependencies(is_recovery_needed)

ament_auto_add_library(is_supply_card_detected SHARED 
    plugins/rmul_2026/condition/is_supply_card_detected.cpp
)
add_target_dependencies(is_supply_card_detected)

ament_auto_add_library(is_control_zone_detected SHARED 
    plugins/rmul_2026/condition/is_control_zone_detected.cpp
)
add_target_dependencies(is_control_zone_detected)

ament_auto_add_library(is_within_scope SHARED 
    plugins/rmul_2026/condition/is_within_scope.cpp
)
add_target_dependencies(is_within_scope)

ament_auto_add_library(is_navigation_stuck SHARED 
    plugins/rmul_2026/condition/is_navigation_stuck.cpp
)
add_target_dependencies(is_navigation_stuck)

ament_auto_add_library(is_in_semantic_zone SHARED 
    plugins/rmul_2026/condition/is_in_semantic_zone.cpp
)
add_target_dependencies(is_in_semantic_zone)
target_link_libraries(is_in_semantic_zone yaml-cpp)

# Actions
ament_auto_add_library(send_goal SHARED 
    plugins/rmul_2026/action/send_goal.cpp
)
add_target_dependencies(send_goal)


ament_auto_add_library(sub_robot_status SHARED 
    plugins/rmul_2026/action/sub_robot_status.cpp
)
add_target_dependencies(sub_robot_status)

ament_auto_add_library(sub_game_status SHARED 
    plugins/rmul_2026/action/sub_game_status.cpp
)
add_target_dependencies(sub_game_status)

## Skipping sub_armors: depends on external package `auto_aim_interfaces` which is
## not present in this workspace. If you add that package, uncomment the
## following lines to enable building `sub_armors`.
# ament_auto_add_library(sub_armors SHARED
#     plugins/action/sub_armors.cpp
# )
# add_target_dependencies(sub_armors)


ament_auto_add_library(get_current_location SHARED 
    plugins/rmul_2026/action/get_current_location.cpp
)
add_target_dependencies(get_current_location)

ament_auto_add_library(move_around SHARED 
    plugins/rmul_2026/action/move_around.cpp
)
add_target_dependencies(move_around)

ament_auto_add_library(robot_control SHARED 
    plugins/rmul_2026/action/robot_control.cpp
)
add_target_dependencies(robot_control)

ament_auto_add_library(nav_control_cmd SHARED
    plugins/rmul_2026/action/nav_control_cmd.cpp
)
add_target_dependencies(nav_control_cmd)

ament_auto_add_library(print_message SHARED 
    plugins/rmul_2026/action/print_message.cpp
)
add_target_dependencies(print_message)

# Sentry follower action plugin (was present in source but missing from build targets)
ament_auto_add_library(sentry_follower SHARED
    plugins/rmul_2026/action/sentry_follower.cpp
)
add_target_dependencies(sentry_follower)



ament_auto_add_library(detect_respawn_and_set_recovery SHARED 
    plugins/rmul_2026/action/detect_respawn_and_set_recovery.cpp
)
add_target_dependencies(detect_respawn_and_set_recovery)

ament_auto_add_library(sub_rfid_status SHARED 
    plugins/rmul_2026/action/sub_rfid_status.cpp
)
add_target_dependencies(sub_rfid_status)

ament_auto_add_library(init_blackboard_config SHARED 
    plugins/rmul_2026/action/init_blackboard_config.cpp
)
add_target_dependencies(init_blackboard_config)

ament_auto_add_library(wait_and_heal SHARED 
    plugins/rmul_2026/action/wait_and_heal.cpp
)
add_target_dependencies(wait_and_heal)

ament_auto_add_library(clear_recovery_flag SHARED 
    plugins/rmul_2026/action/clear_recovery_flag.cpp
)
add_target_dependencies(clear_recovery_flag)

ament_auto_add_library(set_nav_goal_from_config SHARED
    plugins/rmul_2026/action/set_nav_goal_from_config.cpp
)
add_target_dependencies(set_nav_goal_from_config)

ament_auto_add_library(calibrate_center_anchor SHARED
    plugins/rmul_2026/action/calibrate_center_anchor.cpp
)
add_target_dependencies(calibrate_center_anchor)

ament_auto_add_library(set_goal_from_center_offset SHARED
    plugins/rmul_2026/action/set_goal_from_center_offset.cpp
)
add_target_dependencies(set_goal_from_center_offset)

ament_auto_add_library(snap_goal_to_free_space SHARED
    plugins/rmul_2026/action/snap_goal_to_free_space.cpp
)
add_target_dependencies(snap_goal_to_free_space)

ament_auto_add_library(find_escape_point SHARED
    plugins/rmul_2026/action/find_escape_point.cpp
)
add_target_dependencies(find_escape_point)
target_link_libraries(find_escape_point yaml-cpp)

ament_auto_add_library(is_goal_area_clear SHARED
    plugins/rmul_2026/condition/is_goal_area_clear.cpp
)
add_target_dependencies(is_goal_area_clear)

ament_auto_add_library(is_nav_goal_rejected SHARED
    plugins/rmul_2026/condition/is_nav_goal_rejected.cpp
)
add_target_dependencies(is_nav_goal_rejected)

ament_auto_add_library(find_approach_point SHARED
    plugins/rmul_2026/action/find_approach_point.cpp
)
add_target_dependencies(find_approach_point)

ament_auto_add_library(apply_calibration_override SHARED
    plugins/rmul_2026/action/apply_calibration_override.cpp
)
add_target_dependencies(apply_calibration_override)

ament_auto_add_library(execute_nav2_waypoints SHARED
    plugins/rmul_2026/action/execute_nav2_waypoints.cpp
)
add_target_dependencies(execute_nav2_waypoints)

ament_auto_add_library(micro_search_supply_card SHARED
    plugins/rmul_2026/action/micro_search_supply_card.cpp
)
add_target_dependencies(micro_search_supply_card)

ament_auto_add_library(navigate_to_goal SHARED 
    plugins/rmul_2026/condition/navigate_to_goal.cpp
)
add_target_dependencies(navigate_to_goal)

ament_auto_add_library(sub_robot_position SHARED 
    plugins/rmul_2026/action/sub_robot_position.cpp
)
add_target_dependencies(sub_robot_position)

ament_auto_add_library(init_search_timer_if_needed SHARED 
    plugins/rmul_2026/action/init_search_timer_if_needed.cpp
)
add_target_dependencies(init_search_timer_if_needed)

ament_auto_add_library(not_arrived SHARED 
    plugins/rmul_2026/condition/not_arrived.cpp
)
add_target_dependencies(not_arrived)

ament_auto_add_library(is_at_nav_goal SHARED 
    plugins/rmul_2026/condition/is_at_nav_goal.cpp
)
add_target_dependencies(is_at_nav_goal)

ament_auto_add_library(keep_running SHARED 
    plugins/rmul_2026/action/keep_running.cpp
)
add_target_dependencies(keep_running)

ament_auto_add_library(cancel_nav_goal SHARED 
    plugins/rmul_2026/action/cancel_nav_goal.cpp
)
add_target_dependencies(cancel_nav_goal)

ament_auto_add_library(clear_costmap SHARED 
    plugins/rmul_2026/action/clear_costmap.cpp
)
add_target_dependencies(clear_costmap)
target_link_libraries(clear_costmap ${nav2_msgs_TARGETS})

install(PROGRAMS config/RMUC_2026/robot_position_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_position_bridge
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)