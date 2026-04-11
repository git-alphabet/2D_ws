# =============================================================================
# RMUCFieldStatus.msg — 场地增益点/能量机关状态 (裁判系统 0x0101)
# =============================================================================
# 话题: /field_status             频率: 1 Hz
# 来源: 电控/裁判系统串口解析节点
# 订阅: RmucSubFieldStatus → 写入黑板
# 引用: ParseSentryBlackboard → SelectObjective, BaseDefense
# =============================================================================

std_msgs/Header header

# ── 能量机关  ──
uint8  small_energy_status           # bit3-4: 己方小能量机关 (0=未激活, 1=已激活, 2=正在激活)
uint8  big_energy_status             # bit5-6: 己方大能量机关 (0=未激活, 1=已激活, 2=正在激活)

# ── 高地  ──
uint8  central_highland              # bit7-8: 中央高地占领状态 (0=未占领, 1=己方, 2=对方)
uint8  ladder_highland               # bit9-10: 梯形高地占领状态 (0=未占领, 1=己方)

# ── 增益点占领状态  ──
uint8  fortress                      # bit25-26: 己方堡垒增益点 (0=未占领, 1=己方, 2=对方, 3=双方)
uint8  outpost_buff                  # bit27-28: 己方前哨站增益点 (0=未占领, 1=己方, 2=对方)
bool   base_buff                     # bit29: 己方基地增益点, 1=已占领

# # ── 以下字段暂未启用 ──
# bool   supply_no_resource_occupied   # bit0: 己方与资源区不重叠的补给区, 1=已占领
# bool   supply_resource_occupied      # bit1: 己方与资源区重叠的补给区, 1=已占领
# uint16 dart_hit_time                 # bit11-19: 对方飞镖最后击中己方前哨站/基地的时间 (0-420)
# uint8  dart_hit_target               # bit20-22: 击中目标 (0=默认, 1=前哨站, 2-5=基地各区)
