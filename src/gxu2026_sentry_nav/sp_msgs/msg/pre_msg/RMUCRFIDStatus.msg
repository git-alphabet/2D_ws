# =============================================================================
# RMUCRFIDStatus.msg — RFID / 场地交互模块状态 (0x0209)
# =============================================================================
# 话题: /rfid_status           频率: 事件驱动 (变化时发布)
# 来源: 电控/裁判系统串口解析节点
# 订阅: ❌ 已停用 (RmucSubRFIDStatus 插件已删除，RMUCRFIDStatus.msg 所有字段已移除)
# 引用: ❌ 无
# =============================================================================

# std_msgs/Header header

# bool   rfid_supply           # 补给区增益点 RFID 触发
# bool   rfid_base_buff        # 基地增益点 RFID 触发
# bool   rfid_outpost_buff     # 前哨站增益点 RFID 触发
# bool   rfid_central_highland # 中央高地 RFID 触发
# bool   rfid_ladder_highland  # 梯形高地 RFID 触发
# # ── 以下字段暂未启用 ──
# bool   rfid_fortress_enemy   # 敌方堡垒 RFID 触发
