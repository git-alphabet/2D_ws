# =============================================================================
# RMUCSentryCmd.msg — 哨兵自主决策指令 (裁判系统 0x0120)
# =============================================================================
# 话题: /sentry_cmd            频率: 2 Hz
# 方向: BT → 电控
# 发布: SentryCmdMux (CommandHub 子树)
# =============================================================================

std_msgs/Header header

uint8  cmd_posture                  # 姿态 (1=进攻 2=防御 3=移动)
bool   cmd_confirm_respawn          # 确认复活
bool   cmd_confirm_instant_respawn  # 确认兑换立即复活
uint16 cmd_allow_ammo_target        # 允许发弹量目标值 (单调递增)
bool   cmd_trigger_remote_ammo      # 触发远程兑换弹丸 (上升沿有效)
bool   cmd_trigger_remote_hp        # 触发远程兑换血量 (上升沿有效)
# bool   cmd_enable_big_energy        # 大能量机关确认
