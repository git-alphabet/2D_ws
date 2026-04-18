# =============================================================================
# RMUCProjectileAllowance.msg — 允许发弹量与金币 (裁判系统 0x0208)
# =============================================================================
# 话题: /projectile_allowance     频率: 10 Hz
# 来源: 电控/裁判系统串口解析节点
# 订阅: RmucSubProjectileAllowance → 写入黑板
# 引用: ParseSentryBlackboard → DecideEconomyCmd, AmmoPlan
# =============================================================================

std_msgs/Header header

# ── 堡垒储备弹  ──
uint16 fortress_ammo           # offset 6: 堡垒增益点提供的储备 17mm 允许发弹量
                               # 注意: 该值与机器人是否实际占领堡垒无关
uint16 remaining_coins         # offset 4: 剩余金币数量（幽灵端口：仅日志引用，BT 未消费）

# # ── 以下字段暂未启用 ──
# uint16 ammo_17mm               # offset 0: 机器人自身 17mm 允许发弹量（幽灵端口：仅日志引用，BT 未消费）
# uint16 ammo_42mm               # offset 2: 42mm 允许发弹量 (哨兵无42mm)
