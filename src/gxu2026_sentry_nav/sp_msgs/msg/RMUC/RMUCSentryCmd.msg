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
uint16 cmd_allow_ammo_target        # 允许发弹量目标值 (bit2-12, 单调递增)
uint8  cmd_trigger_remote_ammo      # 远程兑换弹丸请求次数 (bit13-16, 4位, 单调递增+1)
uint8  cmd_trigger_remote_hp        # 远程兑换血量请求次数 (bit17-20, 4位, 单调递增+1)
# bool   cmd_enable_big_energy        # 大能量机关确认
